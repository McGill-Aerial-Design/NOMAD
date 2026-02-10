#!/usr/bin/env python3
"""
Task 1 AI Image Description Generator - Dual Provider Edition

Processes captured photos from Task 1 (Outdoor Recon) using AI to generate
detailed scene descriptions including landmarks, localization context,
and relative positioning to competition buildings.

This script runs on the Windows ground station and downloads images from
the Jetson via HTTP API. Supports both cloud (Gemini) and local (Ollama) AI.

Usage:
    # Gemini (cloud)
    python scripts/process_task1_ai.py --provider gemini --gemini-key YOUR_API_KEY
    
    # Ollama (local)
    python scripts/process_task1_ai.py --provider ollama --ollama-model llava:13b
    
    # Specific folder
    python scripts/process_task1_ai.py --provider ollama --folder 20260202_120000
    
    # Upload descriptions back to Jetson
    python scripts/process_task1_ai.py --provider gemini --gemini-key KEY --upload-to-jetson

Requirements:
    pip install requests pillow
    
    For Gemini:
    pip install google-generativeai
    
    For Ollama:
    Install Ollama from https://ollama.ai
    ollama pull llava:13b  # or another vision model

Author: NOMAD Team - McGill Aerial Design
Date: February 2, 2026
"""

import argparse
import base64
import json
import os
import sys
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import requests
from PIL import Image

# Optional imports
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False


# ==================== AI Provider Interface ====================

class AIProvider(ABC):
    """Abstract base class for AI providers."""
    
    @abstractmethod
    def generate_description(self, image_path: Path, metadata: Dict) -> str:
        """
        Generate a detailed description of the image.
        
        Args:
            image_path: Path to the image file
            metadata: Dictionary containing GPS, AHRS, and other metadata
            
        Returns:
            Generated description text
        """
        pass
    
    def _build_prompt(self, metadata: Dict) -> str:
        """Build the detailed prompt based on metadata."""
        gps = metadata.get('gps', {})
        ahrs = metadata.get('ahrs', {})
        building = metadata.get('building_location', 'Unknown')
        
        lat = gps.get('lat', 0.0)
        lon = gps.get('lon', 0.0)
        alt = gps.get('alt', 0.0)
        heading = ahrs.get('heading_deg', 0.0)
        pitch = ahrs.get('pitch_deg', 0.0)
        roll = ahrs.get('roll_deg', 0.0)
        
        prompt = f"""Analyze this outdoor reconnaissance photo with the following context:

GPS Location: {lat:.6f}, {lon:.6f} at {alt:.1f}m altitude
Aircraft Heading: {heading:.1f} degrees (0=North, 90=East, 180=South, 270=West)
Aircraft Attitude: Pitch {pitch:.1f} degrees, Roll {roll:.1f} degrees
Building Location: {building}

Please provide a detailed description including:

1. SCENE DESCRIPTION:
   - Identify all visible landmarks (buildings, roads, natural features)
   - Describe terrain and environment type (urban, rural, industrial, etc.)
   - Note weather conditions and visibility

2. LOCALIZATION FEATURES:
   - Unique identifying features that could be used for position verification
   - Street signs, building numbers, or other text visible in the image
   - Distinctive architectural or natural features

3. RELATIVE POSITION TO BUILDING:
   - Estimate the distance and cardinal direction to the reference building
   - Describe the viewing angle relative to the building
   - Identify any direct visual line-of-sight to the building

4. COMPETITION RELEVANCE:
   - Assess image quality and suitability for evidence submission
   - Note any potential target areas or points of interest
   - Suggest optimal capture angles if this location is revisited

Be precise, factual, and concise. Focus on information useful for autonomous navigation and competition scoring."""
        
        return prompt


class GeminiProvider(AIProvider):
    """Google Gemini AI provider."""
    
    def __init__(self, api_key: str, model: str = "gemini-1.5-flash"):
        """
        Initialize Gemini provider.
        
        Args:
            api_key: Google API key
            model: Model name (gemini-1.5-flash or gemini-1.5-pro)
        """
        if not GEMINI_AVAILABLE:
            raise ImportError("google-generativeai package not installed. Run: pip install google-generativeai")
        
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(model)
        self.model_name = model
    
    def generate_description(self, image_path: Path, metadata: Dict) -> str:
        """Generate description using Gemini."""
        prompt = self._build_prompt(metadata)
        
        # Open image with PIL
        image = Image.open(image_path)
        
        # Call Gemini API
        response = self.model.generate_content([prompt, image])
        
        return response.text


class OllamaProvider(AIProvider):
    """Ollama local AI provider."""
    
    def __init__(self, model: str = "llava:13b", host: str = "http://localhost:11434"):
        """
        Initialize Ollama provider.
        
        Args:
            model: Vision model name (e.g., llava:13b, llava:7b)
            host: Ollama server URL
        """
        self.model = model
        self.host = host.rstrip('/')
        
        # Verify Ollama is running
        try:
            response = requests.get(f"{self.host}/api/tags", timeout=5)
            response.raise_for_status()
        except Exception as e:
            raise ConnectionError(f"Cannot connect to Ollama at {self.host}. Is Ollama running? Error: {e}")
    
    def generate_description(self, image_path: Path, metadata: Dict) -> str:
        """Generate description using Ollama."""
        prompt = self._build_prompt(metadata)
        
        # Encode image as base64
        with open(image_path, 'rb') as f:
            image_data = base64.b64encode(f.read()).decode('utf-8')
        
        # Call Ollama API
        response = requests.post(
            f"{self.host}/api/generate",
            json={
                "model": self.model,
                "prompt": prompt,
                "images": [image_data],
                "stream": False,
            },
            timeout=120,  # Vision models can be slow
        )
        response.raise_for_status()
        
        result = response.json()
        return result.get("response", "")


# ==================== Jetson Integration ====================

def download_capture(
    jetson_ip: str,
    port: int,
    folder: str,
    local_dir: Path,
    verbose: bool = False,
    max_retries: int = 3,
) -> Tuple[bool, Optional[str]]:
    """
    Download a capture folder from Jetson.
    
    Args:
        jetson_ip: Jetson IP address
        port: Jetson API port
        folder: Folder name (e.g., 20260202_120000)
        local_dir: Local directory to save to
        verbose: Print detailed output
        max_retries: Maximum number of retry attempts
        
    Returns:
        Tuple of (success, error_message)
    """
    base_url = f"http://{jetson_ip}:{port}"
    capture_dir = local_dir / folder
    capture_dir.mkdir(parents=True, exist_ok=True)
    
    # Download photo.jpg
    photo_path = capture_dir / "photo.jpg"
    if not photo_path.exists():
        image_url = f"{base_url}/api/task/1/images/{folder}/photo.jpg"
        
        for attempt in range(max_retries):
            try:
                if verbose:
                    print(f"  Downloading photo.jpg (attempt {attempt + 1}/{max_retries})...")
                
                response = requests.get(image_url, timeout=30)
                response.raise_for_status()
                
                with open(photo_path, 'wb') as f:
                    f.write(response.content)
                
                if verbose:
                    print(f"  Photo saved: {photo_path}")
                break
                
            except requests.exceptions.RequestException as e:
                if attempt == max_retries - 1:
                    return False, f"Failed to download photo: {e}"
                time.sleep(2 ** attempt)  # Exponential backoff
    
    # Download metadata.json
    metadata_path = capture_dir / "metadata.json"
    if not metadata_path.exists():
        metadata_url = f"{base_url}/api/task/1/images/{folder}/metadata.json"
        
        for attempt in range(max_retries):
            try:
                if verbose:
                    print(f"  Downloading metadata.json (attempt {attempt + 1}/{max_retries})...")
                
                response = requests.get(metadata_url, timeout=30)
                response.raise_for_status()
                
                with open(metadata_path, 'wb') as f:
                    f.write(response.content)
                
                if verbose:
                    print(f"  Metadata saved: {metadata_path}")
                break
                
            except requests.exceptions.RequestException as e:
                if attempt == max_retries - 1:
                    return False, f"Failed to download metadata: {e}"
                time.sleep(2 ** attempt)  # Exponential backoff
    
    return True, None


def list_captures(
    jetson_ip: str,
    port: int,
    max_retries: int = 3,
) -> Tuple[Optional[List[str]], Optional[str]]:
    """
    List available capture folders on Jetson.
    
    Args:
        jetson_ip: Jetson IP address
        port: Jetson API port
        max_retries: Maximum number of retry attempts
        
    Returns:
        Tuple of (folder_list, error_message)
    """
    base_url = f"http://{jetson_ip}:{port}"
    list_url = f"{base_url}/api/task/1/captures"
    
    for attempt in range(max_retries):
        try:
            response = requests.get(list_url, timeout=10)
            response.raise_for_status()
            
            data = response.json()
            return data.get("captures", []), None
            
        except requests.exceptions.RequestException as e:
            if attempt == max_retries - 1:
                return None, f"Failed to list captures: {e}"
            time.sleep(2 ** attempt)  # Exponential backoff
    
    return None, "Max retries exceeded"


def upload_description(
    jetson_ip: str,
    port: int,
    folder: str,
    description: str,
    max_retries: int = 3,
) -> Tuple[bool, Optional[str]]:
    """
    Upload description back to Jetson.
    
    Args:
        jetson_ip: Jetson IP address
        port: Jetson API port
        folder: Folder name
        description: Description text
        max_retries: Maximum number of retry attempts
        
    Returns:
        Tuple of (success, error_message)
    """
    base_url = f"http://{jetson_ip}:{port}"
    upload_url = f"{base_url}/api/task/1/upload_description"
    
    for attempt in range(max_retries):
        try:
            response = requests.post(
                upload_url,
                json={
                    "folder": folder,
                    "description": description,
                },
                timeout=10,
            )
            response.raise_for_status()
            
            return True, None
            
        except requests.exceptions.RequestException as e:
            if attempt == max_retries - 1:
                return False, f"Failed to upload description: {e}"
            time.sleep(2 ** attempt)  # Exponential backoff
    
    return False, "Max retries exceeded"


# ==================== Processing Logic ====================

def load_metadata(capture_folder: Path) -> Optional[Dict]:
    """Load metadata.json from capture folder."""
    metadata_path = capture_folder / "metadata.json"
    if not metadata_path.exists():
        print(f"  WARNING: No metadata.json found in {capture_folder}")
        return None
    
    try:
        with open(metadata_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"  ERROR: Failed to load metadata from {metadata_path}: {e}")
        return None


def process_capture(
    capture_folder: Path,
    ai_provider: AIProvider,
    jetson_ip: Optional[str] = None,
    jetson_port: Optional[int] = None,
    upload_to_jetson: bool = False,
    verbose: bool = False,
) -> bool:
    """
    Process a single capture folder with AI provider.
    
    Args:
        capture_folder: Path to capture folder
        ai_provider: AI provider instance
        jetson_ip: Jetson IP for uploading (optional)
        jetson_port: Jetson port for uploading (optional)
        upload_to_jetson: Whether to upload description back to Jetson
        verbose: Print detailed output
        
    Returns:
        True if successful, False otherwise
    """
    print(f"\nProcessing: {capture_folder.name}")
    
    # Check for existing description
    desc_path = capture_folder / "description.txt"
    if desc_path.exists():
        print(f"  SKIP: Description already exists")
        return True
    
    # Load metadata
    metadata = load_metadata(capture_folder)
    if not metadata:
        return False
    
    # Load photo
    photo_path = capture_folder / "photo.jpg"
    if not photo_path.exists():
        print(f"  ERROR: No photo.jpg found in {capture_folder}")
        return False
    
    try:
        # Generate description with AI
        if verbose:
            print(f"  Generating description with AI...")
        
        description = ai_provider.generate_description(photo_path, metadata)
        
        # Save description locally
        with open(desc_path, 'w', encoding='utf-8') as f:
            f.write(description)
        
        print(f"  SUCCESS: Description saved to {desc_path.name}")
        
        # Upload to Jetson if requested
        if upload_to_jetson and jetson_ip and jetson_port:
            if verbose:
                print(f"  Uploading description to Jetson...")
            
            success, error = upload_description(
                jetson_ip,
                jetson_port,
                capture_folder.name,
                description,
            )
            
            if success:
                print(f"  SUCCESS: Description uploaded to Jetson")
            else:
                print(f"  WARNING: Failed to upload to Jetson: {error}")
        
        # Print description if verbose
        if verbose:
            print(f"\n--- DESCRIPTION ---")
            print(description)
            print(f"--- END ---\n")
        
        return True
        
    except Exception as e:
        print(f"  ERROR: Failed to process {capture_folder.name}: {e}")
        import traceback
        if verbose:
            traceback.print_exc()
        return False


# ==================== Main Entry Point ====================

def main():
    parser = argparse.ArgumentParser(
        description="Generate AI descriptions for Task 1 captured photos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process with Gemini (cloud)
  python scripts/process_task1_ai.py --provider gemini --gemini-key YOUR_API_KEY
  
  # Process with Ollama (local)
  python scripts/process_task1_ai.py --provider ollama --ollama-model llava:13b
  
  # Process specific folder
  python scripts/process_task1_ai.py --provider ollama --folder 20260202_120000
  
  # Download from Jetson and upload results back
  python scripts/process_task1_ai.py --provider gemini --gemini-key KEY --upload-to-jetson
        """
    )
    
    # Required arguments
    parser.add_argument(
        '--provider',
        required=True,
        choices=['gemini', 'ollama'],
        help='AI provider to use'
    )
    
    # Jetson connection
    parser.add_argument(
        '--jetson-ip',
        default='100.85.121.98',
        help='Jetson IP address (default: 100.85.121.98)'
    )
    parser.add_argument(
        '--jetson-port',
        type=int,
        default=8000,
        help='Jetson API port (default: 8000)'
    )
    
    # Local storage
    parser.add_argument(
        '--local-dir',
        default=r'C:\NOMAD\Task1',
        help=r'Local save directory (default: C:\NOMAD\Task1)'
    )
    
    # Gemini options
    parser.add_argument(
        '--gemini-key',
        help='Google Gemini API key (required if provider=gemini)'
    )
    parser.add_argument(
        '--gemini-model',
        default='gemini-1.5-flash',
        choices=['gemini-1.5-pro', 'gemini-1.5-flash'],
        help='Gemini model (default: gemini-1.5-flash)'
    )
    
    # Ollama options
    parser.add_argument(
        '--ollama-model',
        default='llava:13b',
        help='Ollama vision model (default: llava:13b)'
    )
    parser.add_argument(
        '--ollama-host',
        default='http://localhost:11434',
        help='Ollama server URL (default: http://localhost:11434)'
    )
    
    # Processing options
    parser.add_argument(
        '--folder',
        help='Process only a specific capture folder (e.g., 20260202_120000)'
    )
    parser.add_argument(
        '--upload-to-jetson',
        action='store_true',
        help='Upload descriptions back to Jetson'
    )
    parser.add_argument(
        '--download-only',
        action='store_true',
        help='Only download captures, do not process with AI'
    )
    parser.add_argument(
        '--local-only',
        action='store_true',
        help='Process local captures only, do not download from Jetson'
    )
    
    # Output control
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Print detailed output including generated descriptions'
    )
    
    args = parser.parse_args()
    
    # Validate provider-specific arguments
    if args.provider == 'gemini' and not args.gemini_key:
        parser.error("--gemini-key is required when provider=gemini")
    
    # Initialize AI provider
    if not args.download_only:
        try:
            if args.provider == 'gemini':
                print(f"Initializing Gemini provider with model: {args.gemini_model}")
                ai_provider = GeminiProvider(args.gemini_key, args.gemini_model)
            else:  # ollama
                print(f"Initializing Ollama provider with model: {args.ollama_model}")
                ai_provider = OllamaProvider(args.ollama_model, args.ollama_host)
        except Exception as e:
            print(f"ERROR: Failed to initialize AI provider: {e}")
            sys.exit(1)
    else:
        ai_provider = None
    
    # Create local directory
    local_dir = Path(args.local_dir)
    local_dir.mkdir(parents=True, exist_ok=True)
    print(f"Local directory: {local_dir}")
    
    # Determine which folders to process
    folders_to_process = []
    
    if args.local_only:
        # Process existing local folders
        if args.folder:
            target_folder = local_dir / args.folder
            if not target_folder.exists():
                print(f"ERROR: Local folder not found: {target_folder}")
                sys.exit(1)
            folders_to_process = [target_folder]
        else:
            folders_to_process = [f for f in local_dir.iterdir() if f.is_dir()]
            folders_to_process.sort()
    else:
        # Download from Jetson
        if args.folder:
            # Single folder mode
            print(f"\nDownloading folder: {args.folder}")
            success, error = download_capture(
                args.jetson_ip,
                args.jetson_port,
                args.folder,
                local_dir,
                args.verbose,
            )
            
            if not success:
                print(f"ERROR: {error}")
                sys.exit(1)
            
            folders_to_process = [local_dir / args.folder]
        else:
            # List and download all folders
            print(f"\nListing captures from Jetson {args.jetson_ip}:{args.jetson_port}...")
            
            capture_list, error = list_captures(args.jetson_ip, args.jetson_port)
            
            if error or capture_list is None:
                print(f"WARNING: {error or 'No captures returned'}")
                print(f"Falling back to local folders...")
                folders_to_process = [f for f in local_dir.iterdir() if f.is_dir()]
                folders_to_process.sort()
            else:
                print(f"Found {len(capture_list)} captures on Jetson")
                
                # Download each capture
                for folder_name in capture_list:
                    print(f"\nDownloading folder: {folder_name}")
                    success, error = download_capture(
                        args.jetson_ip,
                        args.jetson_port,
                        folder_name,
                        local_dir,
                        args.verbose,
                    )
                    
                    if success:
                        folders_to_process.append(local_dir / folder_name)
                    else:
                        print(f"  WARNING: {error}")
    
    if not folders_to_process:
        print("No capture folders found to process")
        sys.exit(0)
    
    print(f"\nFound {len(folders_to_process)} folder(s) to process")
    
    # Exit if download-only mode
    if args.download_only:
        print(f"\nDownload complete (download-only mode)")
        sys.exit(0)
    
    # Ensure AI provider is initialized
    if ai_provider is None:
        print(f"ERROR: AI provider not initialized")
        sys.exit(1)
    
    # Process each folder with AI
    success_count = 0
    skip_count = 0
    error_count = 0
    
    for folder in folders_to_process:
        desc_path = folder / "description.txt"
        if desc_path.exists():
            skip_count += 1
            if args.verbose:
                print(f"\nSkipping: {folder.name} (already processed)")
            continue
        
        if process_capture(
            folder,
            ai_provider,
            args.jetson_ip,
            args.jetson_port,
            args.upload_to_jetson,
            args.verbose,
        ):
            success_count += 1
        else:
            error_count += 1
    
    # Summary
    print(f"\n{'='*60}")
    print(f"PROCESSING COMPLETE")
    print(f"{'='*60}")
    print(f"Total folders: {len(folders_to_process)}")
    print(f"Processed: {success_count}")
    print(f"Skipped: {skip_count}")
    print(f"Errors: {error_count}")
    print(f"{'='*60}")
    
    sys.exit(0 if error_count == 0 else 1)


if __name__ == "__main__":
    main()
