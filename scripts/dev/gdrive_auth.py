"""
One-shot OAuth2 flow for Google Drive (installed app).

Usage:
    python scripts/dev/gdrive_auth.py path/to/client_secret.json

The client_secret JSON is the file downloaded from Google Cloud Console
(Application type: Desktop app). It is NOT committed to the repo.

Opens a browser, catches the redirect on localhost, exchanges for tokens,
and writes ~/.nomad/gdrive_token.json in the format GoogleDriveUploadService expects.
"""

import http.server
import json
import sys
import threading
import urllib.parse
import urllib.request
import webbrowser
from pathlib import Path

REDIRECT_PORT = 8765
REDIRECT_URI  = f"http://localhost:{REDIRECT_PORT}"
SCOPE         = "https://www.googleapis.com/auth/drive.file"
FOLDER_ID     = "14cK37Kw6YipYQqaL2l6S-g24SCGhU-xv"
TOKEN_PATH    = Path.home() / ".nomad" / "gdrive_token.json"

auth_code = None
server_done = threading.Event()


class _Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        global auth_code
        parsed = urllib.parse.urlparse(self.path)
        params = urllib.parse.parse_qs(parsed.query)
        if "code" in params:
            auth_code = params["code"][0]
            body = b"<h2>Auth complete! You can close this tab.</h2>"
        else:
            body = f"<pre>Error: {self.path}</pre>".encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(body)
        server_done.set()

    def log_message(self, *_):
        pass


def exchange_code(code: str, client_id: str, client_secret: str) -> dict:
    data = urllib.parse.urlencode({
        "code":          code,
        "client_id":     client_id,
        "client_secret": client_secret,
        "redirect_uri":  REDIRECT_URI,
        "grant_type":    "authorization_code",
    }).encode()
    req = urllib.request.Request(
        "https://oauth2.googleapis.com/token",
        data=data,
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    with urllib.request.urlopen(req) as resp:
        return json.loads(resp.read())


def main():
    if len(sys.argv) < 2:
        print("Usage: python gdrive_auth.py <path-to-client_secret.json>")
        print("Download the client secret from Google Cloud Console (Desktop app type).")
        sys.exit(1)

    secret_path = Path(sys.argv[1])
    if not secret_path.exists():
        print(f"File not found: {secret_path}")
        sys.exit(1)

    raw = json.loads(secret_path.read_text())
    # Supports both {"installed": {...}} and flat format
    creds = raw.get("installed") or raw.get("web") or raw
    client_id     = creds["client_id"]
    client_secret = creds["client_secret"]

    # Start local callback server
    server = http.server.HTTPServer(("localhost", REDIRECT_PORT), _Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()

    # Build auth URL
    params = urllib.parse.urlencode({
        "client_id":     client_id,
        "redirect_uri":  REDIRECT_URI,
        "response_type": "code",
        "scope":         SCOPE,
        "access_type":   "offline",
        "prompt":        "consent",   # force refresh_token to be issued
    })
    url = f"https://accounts.google.com/o/oauth2/auth?{params}"

    print(f"\nOpening browser for Google auth...")
    print(f"If it doesn't open, visit:\n  {url}\n")
    webbrowser.open(url)

    server_done.wait(timeout=120)
    server.shutdown()

    if not auth_code:
        print("ERROR: No auth code received (timed out).")
        sys.exit(1)

    print("Auth code received, exchanging for tokens...")
    tokens = exchange_code(auth_code, client_id, client_secret)

    if "refresh_token" not in tokens:
        print("ERROR: No refresh_token in response. Try revoking access at "
              "https://myaccount.google.com/permissions and re-running.")
        print("Response:", json.dumps(tokens, indent=2))
        sys.exit(1)

    token_data = {
        "access_token":  tokens["access_token"],
        "refresh_token": tokens["refresh_token"],
        "client_id":     client_id,
        "client_secret": client_secret,
        "token_type":    tokens.get("token_type", "Bearer"),
        "expires_in":    tokens.get("expires_in", 3600),
        "scope":         tokens.get("scope", SCOPE),
        "folder_id":     FOLDER_ID,
    }

    TOKEN_PATH.parent.mkdir(parents=True, exist_ok=True)
    TOKEN_PATH.write_text(json.dumps(token_data, indent=2))
    print(f"\nToken saved to: {TOKEN_PATH}")
    print("Google Drive upload is now configured.")


if __name__ == "__main__":
    main()
