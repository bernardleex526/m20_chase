"""Minimal RFC6455 WebSocket server (stdlib only), enough for the web UI.

Same role as jie_deamon's hand-rolled WebSocket: handshake + text frames,
broadcast to all clients, and a per-message callback.
"""

import base64
import hashlib
import socket
import struct
import threading

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class _Client:
    def __init__(self, conn, addr):
        self.conn = conn
        self.addr = addr
        self.alive = True
        self.lock = threading.Lock()

    def send_text(self, s):
        data = s.encode('utf-8')
        hdr = bytearray([0x81])
        n = len(data)
        if n < 126:
            hdr.append(n)
        elif n < 65536:
            hdr.append(126)
            hdr += struct.pack('>H', n)
        else:
            hdr.append(127)
            hdr += struct.pack('>Q', n)
        with self.lock:
            try:
                self.conn.sendall(bytes(hdr) + data)
            except OSError:
                self.alive = False

    def _recv(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.conn.recv(n - len(buf))
            if not chunk:
                raise ConnectionError
            buf += chunk
        return buf

    def recv_frame(self):
        b1, b2 = self._recv(2)
        opcode = b1 & 0x0F
        masked = b2 & 0x80
        length = b2 & 0x7F
        if length == 126:
            length = struct.unpack('>H', self._recv(2))[0]
        elif length == 127:
            length = struct.unpack('>Q', self._recv(8))[0]
        mask = self._recv(4) if masked else b'\x00' * 4
        data = self._recv(length) if length else b''
        if masked:
            data = bytes(d ^ mask[i % 4] for i, d in enumerate(data))
        return opcode, data


class WSServer:
    def __init__(self, port, on_message):
        self.on_message = on_message
        self.clients = []
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.listen(8)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                break
            threading.Thread(target=self._handle, args=(conn, addr), daemon=True).start()

    def _handle(self, conn, addr):
        client = None
        try:
            data = b''
            while b'\r\n\r\n' not in data:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                data += chunk
            key = None
            for line in data.decode('utf-8', 'ignore').split('\r\n'):
                if line.lower().startswith('sec-websocket-key:'):
                    key = line.split(':', 1)[1].strip()
            if not key:
                return
            accept = base64.b64encode(
                hashlib.sha1((key + GUID).encode()).digest()).decode()
            conn.sendall((
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
            client = _Client(conn, addr)
            with self.lock:
                self.clients.append(client)
            while client.alive:
                op, payload = client.recv_frame()
                if op == 0x8:
                    break
                if op in (0x1, 0x2):
                    try:
                        self.on_message(payload.decode('utf-8'))
                    except Exception:
                        pass
        except Exception:
            pass
        finally:
            if client is not None:
                client.alive = False
                with self.lock:
                    if client in self.clients:
                        self.clients.remove(client)
            try:
                conn.close()
            except OSError:
                pass

    def broadcast(self, text):
        with self.lock:
            clients = list(self.clients)
        for c in clients:
            if c.alive:
                c.send_text(text)

    def client_count(self):
        with self.lock:
            return len(self.clients)
