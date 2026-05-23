import pytest
import struct
import socket
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.request import urlopen, Request
from urllib.error import HTTPError, URLError


# Simulate a protected endpoint server that mimics the WiFi frame receive path
# with authentication checks before processing packet data

class MockProtectedHandler(BaseHTTPRequestHandler):
    """Mock server simulating a protected endpoint for WiFi frame processing."""

    def log_message(self, format, *args):
        pass  # Suppress output

    def do_POST(self):
        auth_header = self.headers.get('Authorization', '')
        token = self.headers.get('X-Auth-Token', '')
        api_key = self.headers.get('X-API-Key', '')

        # Only accept a specific valid token for testing purposes
        valid_token = "valid_secret_token_12345"

        if not self._is_authenticated(auth_header, token, api_key, valid_token):
            self.send_response(401)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"error": "Unauthorized"}')
            return

        # If authenticated, process the frame (would normally call memcpy path)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"status": "ok"}')

    def do_GET(self):
        self.do_POST()

    def _is_authenticated(self, auth_header, token, api_key, valid_token):
        if auth_header == f"Bearer {valid_token}":
            return True
        if token == valid_token:
            return True
        if api_key == valid_token:
            return True
        return False


def craft_malicious_radiotap_frame(radiotap_length, payload_size=1024):
    """Craft a malicious 802.11 radiotap frame with manipulated length fields."""
    # Radiotap header: version, pad, length (little-endian), present flags
    if radiotap_length == 0:
        header = struct.pack('<BBHI', 0, 0, 0, 0)
    else:
        header = struct.pack('<BBHI', 0, 0, radiotap_length, 0)
    payload = b'\xAA' * payload_size
    return header + payload


@pytest.fixture(scope="module")
def protected_server():
    """Start a mock protected server for testing."""
    server = HTTPServer(('127.0.0.1', 0), MockProtectedHandler)
    port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()
    time.sleep(0.1)
    yield f"http://127.0.0.1:{port}"
    server.shutdown()


# Adversarial payloads: (description, headers, body)
UNAUTHENTICATED_PAYLOADS = [
    # Missing authentication entirely
    {
        "description": "no_auth_headers",
        "headers": {},
        "body": craft_malicious_radiotap_frame(0),  # radiotap_length=0 triggers overflow
    },
    # Expired/fake Bearer token
    {
        "description": "expired_bearer_token",
        "headers": {"Authorization": "Bearer expired_token_xyz_000"},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Malformed Bearer token (SQL injection attempt)
    {
        "description": "sql_injection_token",
        "headers": {"Authorization": "Bearer ' OR '1'='1"},
        "body": craft_malicious_radiotap_frame(1),  # very small radiotap_length
    },
    # Malformed Bearer token (null bytes)
    {
        "description": "null_byte_token",
        "headers": {"Authorization": "Bearer \x00\x00\x00"},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Empty Bearer token
    {
        "description": "empty_bearer_token",
        "headers": {"Authorization": "Bearer "},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Wrong scheme
    {
        "description": "wrong_auth_scheme",
        "headers": {"Authorization": "Basic dXNlcjpwYXNz"},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Malformed X-Auth-Token (path traversal attempt)
    {
        "description": "path_traversal_token",
        "headers": {"X-Auth-Token": "../../etc/passwd"},
        "body": craft_malicious_radiotap_frame(2),
    },
    # Malformed X-Auth-Token (XSS attempt)
    {
        "description": "xss_token",
        "headers": {"X-Auth-Token": "<script>alert(1)</script>"},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Malformed X-API-Key (buffer overflow string)
    {
        "description": "overflow_api_key",
        "headers": {"X-API-Key": "A" * 65536},
        "body": craft_malicious_radiotap_frame(0),
    },
    # JWT with tampered signature
    {
        "description": "tampered_jwt",
        "headers": {"Authorization": "Bearer eyJhbGciOiJub25lIn0.eyJ1c2VyIjoiYWRtaW4ifQ."},
        "body": craft_malicious_radiotap_frame(0),
    },
    # JWT algorithm confusion (alg=none)
    {
        "description": "jwt_alg_none",
        "headers": {"Authorization": "Bearer eyJhbGciOiJOT05FIn0.eyJzdWIiOiIxMjM0NTY3ODkwIn0."},
        "body": craft_malicious_radiotap_frame(1),
    },
    # Whitespace-only token
    {
        "description": "whitespace_token",
        "headers": {"X-Auth-Token": "   "},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Unicode token
    {
        "description": "unicode_token",
        "headers": {"X-Auth-Token": "\u202e\u0000\uffff"},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Crafted frame with radiotap_length=0 and no auth
    {
        "description": "radiotap_zero_length_no_auth",
        "headers": {},
        "body": craft_malicious_radiotap_frame(radiotap_length=0, payload_size=4096),
    },
    # Crafted frame with very small radiotap_length and no auth
    {
        "description": "radiotap_tiny_length_no_auth",
        "headers": {},
        "body": craft_malicious_radiotap_frame(radiotap_length=1, payload_size=4096),
    },
    # Authorization header with only scheme
    {
        "description": "auth_scheme_only",
        "headers": {"Authorization": "Bearer"},
        "body": craft_malicious_radiotap_frame(0),
    },
    # Multiple conflicting auth headers (only first should matter)
    {
        "description": "conflicting_auth_headers",
        "headers": {
            "Authorization": "Bearer invalid_token",
            "X-Auth-Token": "also_invalid",
            "X-API-Key": "still_invalid",
        },
        "body": craft_malicious_radiotap_frame(0),
    },
]


@pytest.mark.parametrize("payload", UNAUTHENTICATED_PAYLOADS, ids=[p["description"] for p in UNAUTHENTICATED_PAYLOADS])
def test_protected_endpoint_rejects_unauthenticated_requests(payload, protected_server):
    """
    Invariant: Protected endpoints MUST reject all unauthenticated or invalidly
    authenticated requests with HTTP 401 or 403, regardless of the packet payload
    content. This prevents unauthenticated access to the WiFi frame processing path
    (CWE-287), which could otherwise allow attackers to trigger memory corruption
    via crafted radiotap frames with manipulated length fields without any
    authentication required.
    """
    url = f"{protected_server}/api/frame/receive"
    body = payload["body"]
    headers = {
        "Content-Type": "application/octet-stream",
        "Content-Length": str(len(body)),
    }
    headers.update(payload["headers"])

    # Filter out headers with non-ASCII values that urllib can't handle
    safe_headers = {}
    for k, v in headers.items():
        try:
            k.encode('ascii')
            v.encode('latin-1')
            safe_headers[k] = v
        except (UnicodeEncodeError, UnicodeDecodeError):
            # Skip headers that can't be encoded; they won't authenticate anyway
            pass

    req = Request(url, data=body, headers=safe_headers, method='POST')

    try:
        response = urlopen(req, timeout=5)
        # If we get here, the server returned 2xx — this is a security violation
        status_code = response.status
        pytest.fail(
            f"SECURITY VIOLATION: Unauthenticated request with payload "
            f"'{payload['description']}' was accepted with status {status_code}. "
            f"Protected endpoints must reject unauthenticated requests with 401 or 403."
        )
    except HTTPError as e:
        # Expected: server should return 401 or 403
        assert e.code in (401, 403), (
            f"Expected 401 or 403 for unauthenticated request "
            f"'{payload['description']}', but got {e.code}. "
            f"This may indicate improper authentication enforcement (CWE-287)."
        )
    except URLError as e:
        pytest.skip(f"Server connection error for payload '{payload['description']}': {e}")


@pytest.mark.parametrize("payload", UNAUTHENTICATED_PAYLOADS, ids=[p["description"] for p in UNAUTHENTICATED_PAYLOADS])
def test_protected_endpoint_rejects_unauthenticated_get_requests(payload, protected_server):
    """
    Invariant: Protected endpoints MUST reject unauthenticated GET requests with
    HTTP 401 or 403. No HTTP method should bypass authentication on protected
    endpoints (CWE-287).
    """
    url = f"{protected_server}/api/frame/receive"
    headers = {"Content-Type": "application/octet-stream"}
    headers.update(payload["headers"])

    safe_headers = {}
    for k, v in headers.items():
        try:
            k.encode('ascii')
            v.encode('latin-1')
            safe_headers[k] = v
        except (UnicodeEncodeError, UnicodeDecodeError):
            pass

    req = Request(url, headers=safe_headers, method='GET')

    try:
        response = urlopen(req, timeout=5)
        status_code = response.status
        pytest.fail(
            f"SECURITY VIOLATION: Unauthenticated GET request with payload "
            f"'{payload['description']}' was accepted with status {status_code}. "
            f"Protected endpoints must reject unauthenticated requests with 401 or 403."
        )
    except HTTPError as e:
        assert e.code in (401, 403), (
            f"Expected 401 or 403 for unauthenticated GET request "
            f"'{payload['description']}', but got {e.code}."
        )
    except URLError as e:
        pytest.skip(f"Server connection error for payload '{payload['description']}': {e}")