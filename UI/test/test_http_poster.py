"""HttpPoster against a real local HTTP server (stdlib only, loopback, no network needed)."""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from ui.http_poster import HttpPoster


class Handler(BaseHTTPRequestHandler):
    received = []
    status = 200

    def do_POST(self):
        body = self.rfile.read(int(self.headers["Content-Length"]))
        Handler.received.append((self.path, self.headers["Content-Type"], json.loads(body)))
        self.send_response(Handler.status)
        self.end_headers()
        self.wfile.write(b"{}")

    def log_message(self, *args):  # keep test output quiet
        pass


@pytest.fixture
def server():
    Handler.received = []
    Handler.status = 200
    httpd = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{httpd.server_port}/api/telemetry"
    httpd.shutdown()
    httpd.server_close()


def test_posts_json(server):
    HttpPoster(server, 2.0).post({"latitude": 1.5, "longitude": -2.5, "altitude_m": 3.0})
    path, content_type, body = Handler.received[0]
    assert path == "/api/telemetry"
    assert content_type == "application/json"
    assert body == {"latitude": 1.5, "longitude": -2.5, "altitude_m": 3.0}


def test_http_error_raises_oserror_with_status(server):
    Handler.status = 422
    with pytest.raises(OSError, match="422"):
        HttpPoster(server, 2.0).post({"latitude": 91.0})


def test_connection_refused_raises_oserror():
    with pytest.raises(OSError):
        HttpPoster("http://127.0.0.1:1/api/telemetry", 0.5).post({})


@pytest.mark.parametrize("url", ["file:///etc/passwd", "ftp://host/x", "127.0.0.1:8000", ""])
def test_only_http_urls_are_accepted(url):
    with pytest.raises(ValueError):
        HttpPoster(url, 1.0)
