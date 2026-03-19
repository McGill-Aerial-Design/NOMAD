"""
NOMAD Edge Core - Google Drive Upload (SP-007)

Uploads spray verification photos to Google Drive using OAuth2 user credentials.

Setup (one-time):
    1. Create OAuth2 credentials (Desktop app) in Google Cloud Console
    2. Download the client_secret JSON
    3. Run: python -m edge_core.gdrive_upload --setup <client_secret.json>
    4. Browser opens, sign in and authorize
    5. Token saved to ~/.nomad/gdrive_token.json (auto-refreshes)

Configuration via environment variables:
    GDRIVE_CREDENTIALS_PATH - Path to OAuth2 client secret JSON (default: ~/.nomad/gdrive_credentials.json)
    GDRIVE_TOKEN_PATH       - Path to saved token (default: ~/.nomad/gdrive_token.json)
    GDRIVE_FOLDER_ID        - Target Google Drive folder ID

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import json
import logging
import os
from typing import Optional

logger = logging.getLogger("edge_core.gdrive_upload")

_SCOPES = ["https://www.googleapis.com/auth/drive.file"]


def _get_token_path() -> str:
    return os.environ.get(
        "GDRIVE_TOKEN_PATH",
        os.path.expanduser("~/.nomad/gdrive_token.json"),
    )


def _get_credentials():
    """Load OAuth2 user credentials from saved token, refreshing if needed."""
    token_path = _get_token_path()

    if not os.path.isfile(token_path):
        return None

    try:
        from google.oauth2.credentials import Credentials
        from google.auth.transport.requests import Request

        creds = Credentials.from_authorized_user_file(token_path, _SCOPES)
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
            # Save refreshed token
            with open(token_path, "w") as f:
                f.write(creds.to_json())
        return creds
    except Exception as e:
        logger.error(f"Failed to load/refresh Google credentials: {e}")
        return None


def setup_credentials(client_secret_path: str) -> bool:
    """
    Run interactive OAuth2 flow to get user credentials.

    Opens a browser for Google sign-in. Saves the resulting token
    to ~/.nomad/gdrive_token.json for headless use.

    Args:
        client_secret_path: Path to OAuth2 client_secret JSON from Google Cloud Console.

    Returns:
        True on success.
    """
    try:
        from google_auth_oauthlib.flow import InstalledAppFlow
    except ImportError:
        print("ERROR: pip install google-auth-oauthlib")
        return False

    flow = InstalledAppFlow.from_client_secrets_file(client_secret_path, _SCOPES)
    creds = flow.run_local_server(port=0)

    token_path = _get_token_path()
    os.makedirs(os.path.dirname(token_path), exist_ok=True)
    with open(token_path, "w") as f:
        f.write(creds.to_json())
    os.chmod(token_path, 0o600)

    print(f"Token saved to {token_path}")
    return True


def upload_to_gdrive(
    local_path: str,
    filename: str,
    folder_id: Optional[str] = None,
) -> str:
    """
    Upload a file to Google Drive via OAuth2 user credentials.

    Args:
        local_path: Path to the local file to upload.
        filename: Name for the file in Google Drive.
        folder_id: Google Drive folder ID. Falls back to GDRIVE_FOLDER_ID env var.

    Returns:
        The Google Drive file ID on success, or empty string on failure.
    """
    folder_id = folder_id or os.environ.get("GDRIVE_FOLDER_ID", "")

    if not os.path.isfile(local_path):
        logger.warning(f"Local file not found: {local_path}")
        return ""

    try:
        from googleapiclient.discovery import build
        from googleapiclient.http import MediaFileUpload
    except ImportError:
        logger.warning(
            "google-api-python-client not installed — "
            "run: pip install google-api-python-client google-auth google-auth-oauthlib"
        )
        return ""

    creds = _get_credentials()
    if creds is None:
        logger.warning(
            f"No Google Drive token at {_get_token_path()} — "
            "run: python -m edge_core.gdrive_upload --setup <client_secret.json>"
        )
        return ""

    try:
        service = build("drive", "v3", credentials=creds)

        file_metadata: dict = {"name": filename}
        if folder_id:
            file_metadata["parents"] = [folder_id]

        media = MediaFileUpload(local_path, resumable=False)
        result = (
            service.files()
            .create(
                body=file_metadata,
                media_body=media,
                fields="id",
                supportsAllDrives=True,
            )
            .execute()
        )

        file_id = result.get("id", "")
        logger.info(f"Uploaded {filename} to Google Drive (id={file_id})")
        return file_id

    except Exception as e:
        logger.error(f"Google Drive upload failed: {e}")
        return ""


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="NOMAD Google Drive upload setup")
    parser.add_argument(
        "--setup",
        metavar="CLIENT_SECRET_JSON",
        help="Run OAuth2 setup flow with the given client_secret JSON file",
    )
    args = parser.parse_args()

    if args.setup:
        setup_credentials(args.setup)
    else:
        parser.print_help()
