"""Opt-in checks against an actual running ROS WebUi, never a fake service.

WEB_UI_HTTP=http://127.0.0.1:8080 WEB_UI_TOKEN=... python3 -m pytest -q test_web_http_live.py
The process under test must already be running; use an isolated ROS domain.
"""
import json
import os
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

BASE = os.environ.get('WEB_UI_HTTP')
pytestmark = pytest.mark.skipif(not BASE, reason='WEB_UI_HTTP must name a running ROS WebUi')


def request(path, payload=None, token=None, origin=None):
    headers = {}
    if token is not None:
        headers['Authorization'] = 'Bearer ' + token
    if origin:
        headers['Origin'] = origin
    data = json.dumps(payload).encode() if payload is not None else None
    try:
        with urlopen(Request(BASE + path, data=data, headers=headers), timeout=2) as response:
            return response.status, response.read()
    except HTTPError as error:
        return error.code, error.read()


def test_live_http_security_and_validation():
    token = os.environ.get('WEB_UI_TOKEN', '')
    code, body = request('/api/config')
    assert code == 200 and isinstance(json.loads(body)['ws_port'], int)
    assert request('/api/status')[0] == 401
    assert request('/api/status', token=token + 'wrong')[0] == 401
    assert request('/api/status', token=token, origin='http://untrusted.invalid')[0] == 403
    code, body = request('/api/status', token=token)
    assert code == 200
    state = json.loads(body)
    assert state['type'] == 'state' and type(state['estop']) is bool
    assert request('/api/enable', {'value': 'false'}, token)[0] == 400
    assert request('/api/mode', {'value': True}, token)[0] == 400
    assert request('/api/direct', {'vx': float('nan'), 'vy': 0, 'wz': 0}, token)[0] == 400
    assert request('/api/enable', {'value': True}, token)[0] == 403
    for path in ('/%2e%2e/package.xml', '/%2e%2e/web-other/index.html', '/%2fetc/passwd'):
        assert request(path)[0] == 404
    # This is an actual software pause; the live node receives enable=false + zero direct.
    code, body = request('/api/enable', {'value': False}, token)
    assert code == 200 and json.loads(body)['success'] is True
