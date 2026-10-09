"""Async websocket transport with authenticated, fail-closed operator leases."""
import asyncio
import hmac
import json
import threading
import time
from collections import deque

try:
    from websockets.asyncio.server import serve
except ImportError:
    from websockets.legacy.server import serve
from websockets.exceptions import ConnectionClosed


class _Client:
    def __init__(self, socket):
        self.socket = socket
        self.event = asyncio.Event()
        self.results = deque(maxlen=64)
        self.state = None
        self.cloud = None
        self.last_cloud = None


class WSServer:
    def __init__(self, port, on_message, *, host, token, origins, on_release):
        self.on_message = on_message
        self.on_release = on_release
        self.token = token
        self.origins = origins
        self.host, self.port = host, port
        self.lock = threading.RLock()
        self.clients = set()
        self.owner = None
        self.deadline = 0.0
        self.latest_state = None
        self.latest_cloud = None
        self.update_pending = False
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.ready.wait()
        if self.error:
            raise self.error

    def _run(self):
        try:
            asyncio.run(self._serve())
        except Exception as exc:
            self.error = exc
            self.ready.set()

    async def _serve(self):
        self.loop = asyncio.get_running_loop()
        self.shutdown = asyncio.Event()
        async with serve(self._handle, self.host, self.port,
                         origins=self.origins, max_size=4096, max_queue=16,
                         compression=None, close_timeout=0.2) as server:
            self.bound_port = server.sockets[0].getsockname()[1]
            self.ready.set()
            watchdog = asyncio.create_task(self._watchdog())
            try:
                await self.shutdown.wait()
            finally:
                watchdog.cancel()
                await asyncio.gather(watchdog, return_exceptions=True)
                with self.lock:
                    self._release()

    def authenticate(self, token):
        return isinstance(token, str) and hmac.compare_digest(
            token.encode('utf-8'), self.token.encode('utf-8'))

    def _release(self):
        if self.owner is not None:
            self.owner = None
            self.deadline = 0.0
            self.on_release()
            for client in self.clients:
                if client.state is not None:
                    client.state['controller'] = False
                client.results.append({'type': 'lease', 'controller': False})
                client.event.set()

    async def _watchdog(self):
        while True:
            await asyncio.sleep(0.05)
            with self.lock:
                if self.owner is not None and time.monotonic() >= self.deadline:
                    old = self.owner
                    self._release()
                    asyncio.create_task(old.socket.close(code=1008, reason='heartbeat expired'))

    async def _handle(self, socket):
        client = _Client(socket)
        sender = None
        try:
            raw = await asyncio.wait_for(socket.recv(), 3.0)
            message = json.loads(raw) if isinstance(raw, str) else None
            if not isinstance(message, dict) or message.get('type') != 'auth' or not self.authenticate(message.get('token')):
                await socket.close(code=1008, reason='authentication required')
                return
            with self.lock:
                self.clients.add(client)
                if self.owner is None:
                    # Every newly acquired lease starts from a paused command state.
                    self.on_release()
                    self.owner = client
                    self.deadline = time.monotonic() + 0.5
                client.results.append({'type': 'auth', 'success': True,
                                       'controller': self.owner is client})
                if self.latest_state is not None:
                    client.state = dict(self.latest_state, controller=self.owner is client, authenticated=True)
                client.cloud = self.latest_cloud
                client.last_cloud = self.latest_cloud
                client.event.set()
            sender = asyncio.create_task(self._send(client))
            async for raw in socket:
                try:
                    message = json.loads(raw) if isinstance(raw, str) else None
                    if not isinstance(message, dict):
                        raise ValueError('BAD_MESSAGE')
                    with self.lock:
                        if self.owner is client and time.monotonic() >= self.deadline:
                            self._release()
                        if message.get('type') == 'heartbeat':
                            if self.owner is client:
                                self.deadline = time.monotonic() + 0.5
                            continue
                        if self.owner is not client:
                            result = {'success': False, 'reason': 'READ_ONLY'}
                        else:
                            result = self.on_message(message, client)
                    self.reply(client, dict(result, type='result', action=message.get('type')))
                except (ValueError, TypeError, json.JSONDecodeError) as exc:
                    self.reply(client, {'type': 'result', 'success': False, 'reason': str(exc)})
        except (ConnectionClosed, asyncio.TimeoutError, ValueError, TypeError):
            await socket.close(code=1008, reason='invalid or expired authentication')
        finally:
            with self.lock:
                self.clients.discard(client)
                if self.owner is client:
                    self._release()
            if sender is not None:
                sender.cancel()
                await asyncio.gather(sender, return_exceptions=True)

    async def _send(self, client):
        try:
            while True:
                await client.event.wait()
                client.event.clear()
                with self.lock:
                    results = list(client.results)
                    client.results.clear()
                    state, cloud = client.state, client.cloud
                    client.state = client.cloud = None
                for message in results + ([state] if state is not None else []):
                    await asyncio.wait_for(client.socket.send(json.dumps(message, allow_nan=False)), 0.2)
                if cloud is not None:
                    await asyncio.wait_for(client.socket.send(cloud), 0.2)
        except (asyncio.TimeoutError, ConnectionClosed):
            with self.lock:
                if self.owner is client:
                    self._release()
            await client.socket.close(code=1008, reason='slow client')

    def reply(self, client, result):
        def deliver():
            with self.lock:
                if client in self.clients:
                    client.results.append(result)
                    client.event.set()
        self.loop.call_soon_threadsafe(deliver)

    def broadcast(self, state, cloud=None):
        with self.lock:
            self.latest_state = state
            self.latest_cloud = cloud
            if self.update_pending:
                return
            self.update_pending = True
        self.loop.call_soon_threadsafe(self._distribute)

    def _distribute(self):
        with self.lock:
            self.update_pending = False
            for client in self.clients:
                client.state = dict(self.latest_state, controller=self.owner is client, authenticated=True)
                if self.latest_cloud is not client.last_cloud:
                    client.cloud = self.latest_cloud
                    client.last_cloud = self.latest_cloud
                client.event.set()

    def client_count(self):
        with self.lock:
            return len(self.clients)

    def close(self):
        self.loop.call_soon_threadsafe(self.shutdown.set)
        self.thread.join(timeout=2.0)
