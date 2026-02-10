#!/usr/bin/env python3
"""
DEPRECATED: This script is superseded by process_task1_ai.py which supports
multiple AI providers (Gemini, Ollama). Use that script instead:
    python scripts/process_task1_ai.py --provider gemini --gemini-key YOUR_KEY

---

Task 1 AI Image Description Generator

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

Requirements:
    pip install requests pillow Pillow
    
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


def load_metadata(capture_folder: Path) -> Optional[Dict]:
    """Load metadata.json from capture folder."""
    metadata_path = capture_folder / "metadata.json"
    if not metadata_path.exists():
        print(f"WARNING: No metadata.json found in {capture_folder}")
        return None
    
    try:
        with open(metadata_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"ERROR: Failed to load metadata from {metadata_path}: {e}")
        return None


def generate_gemini_prompt(metadata: Dict) -> str:
    """Generate detailed prompt for Gemini based on metadata."""
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


def process_capture(capture_folder: Path, model, verbose: bool = False) -> bool:
    """Process a single capture folder with Gemini AI."""
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
        # Open image with PIL
        image = Image.open(photo_path)
        if verbose:
            print(f"  Image size: {image.size}")
        
        # Generate prompt
        prompt = generate_gemini_prompt(metadata)
        
        # Call Gemini API
        if verbose:
            print(f"  Calling Gemini API...")
        response = model.generate_content([prompt, image])
        
        # Save description
        with open(desc_path, 'w', encoding='utf-8') as f:
            f.write(response.text)
        
        print(f"  SUCCESS: Description saved to {desc_path.name}")
        
        # Optionally print description
        if verbose:
            print(f"\n--- DESCRIPTION ---")
            print(response.text)
            print(f"--- END ---\n")
        
        return True
        
    except Exception as e:
        print(f"  ERROR: Failed to process {capture_folder.name}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Generate AI descriptions for Task 1 captured photos using Google Gemini"
    )
    parser.add_argument(
        '--gemini-key',
        required=True,
        help='Google Gemini API key'
    )
    parser.add_argument(
        '--captures-dir',
        default='data/task1_captures',
        help='Path to task1_captures directory (default: data/task1_captures)'
    )
    parser.add_argument(
        '--folder',
        help='Process only a specific capture folder (e.g., 20260202_120000)'
    )
    parser.add_argument(
        '--model',
        default='gemini-1.5-flash',
        choices=['gemini-1.5-pro', 'gemini-1.5-flash'],
        help='Gemini model to use (default: gemini-1.5-flash for cost efficiency)'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Print detailed output including generated descriptions'
    )
    
    args = parser.parse_args()
    
    # Configure Gemini API
    print(f"Configuring Gemini API with model: {args.model}")
    genai.configure(api_key=args.gemini_key)
    model = genai.GenerativeModel(args.model)
    
    # Locate captures directory
    captures_dir = Path(args.captures_dir)
    if not captures_dir.exists():
        print(f"ERROR: Captures directory not found: {captures_dir}")
        sys.exit(1)
    
    # Determine which folders to process
    if args.folder:
        # Single folder mode
        target_folder = captures_dir / args.folder
        if not target_folder.exists():
            print(f"ERROR: Capture folder not found: {target_folder}")
            sys.exit(1)
        folders = [target_folder]
    else:
        # Batch mode - all folders
        folders = [f for f in captures_dir.iterdir() if f.is_dir()]
        folders.sort()  # Process in chronological order
    
    if not folders:
        print(f"No capture folders found in {captures_dir}")
        sys.exit(0)
    
    print(f"Found {len(folders)} capture folder(s) to process")
    
    # Process each folder
    success_count = 0
    skip_count = 0
    error_count = 0
    
    for folder in folders:
        desc_path = folder / "description.txt"
        if desc_path.exists():
            skip_count += 1
            if args.verbose:
                print(f"\nSkipping: {folder.name} (already processed)")
            continue
        
        if process_capture(folder, model, args.verbose):
            success_count += 1
        else:
            error_count += 1
    
    # Summary
    print(f"\n{'='*60}")
    print(f"PROCESSING COMPLETE")
    print(f"{'='*60}")
    print(f"Total folders: {len(folders)}")
    print(f"Processed: {success_count}")
    print(f"Skipped: {skip_count}")
    print(f"Errors: {error_count}")
    print(f"{'='*60}")
    
    sys.exit(0 if error_count == 0 else 1)


if __name__ == "__main__":
    main()
