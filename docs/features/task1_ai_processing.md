# Task 1 AI-Powered Scene Description

**Feature Category**: Task 1 - Outdoor Reconnaissance AI Processing  
**Status**: Production Ready  
**Last Updated**: February 2, 2026

## Overview

Automated scene description generation for Task 1 captured photos using AI. Supports both cloud (Gemini) and local (Ollama) processing.

**Key Features**:
- Runs on Windows ground station (not Jetson)
- Dual AI provider support (Gemini or Ollama)
- Downloads captures from Jetson via HTTP API
- Generates detailed scene descriptions
- Optionally uploads descriptions back to Jetson

---

## Architecture

### Processing Flow
```
Windows Ground Station
        ↓
  Download captures from Jetson
  (HTTP API: /api/task/1/captures)
        ↓
  Process images with AI
  (Gemini Cloud OR Ollama Local)
        ↓
  Generate scene description
        ↓
  Save locally (C:\NOMAD\Task1\{timestamp}\description.txt)
        ↓
  [Optional] Upload to Jetson
  (HTTP API: /api/task/1/upload_description)
```

###  Output Structure
```
Jetson:
  data/task1_captures/
    └── {timestamp}/
        ├── photo.jpg
        ├── metadata.json
        └── description.txt  (uploaded from Windows)

Windows:
  C:\NOMAD\Task1\
    └── {timestamp}/
        ├── photo.jpg        (downloaded)
        ├── metadata.json    (downloaded)
        └── description.txt  (generated)
```

---

## AI Providers

### Gemini (Cloud)
**Pros**:
- Fast (1-2 seconds per image)
- High quality descriptions
- No local setup required

**Cons**:
- Requires API key
- Sends images to Google servers
- Cost: ~$0.001 per image (~$0.10 per 100 images)

**Setup**:
```powershell
pip install google-generativeai
$env:GEMINI_API_KEY="your-api-key-here"
```

### Ollama (Local)
**Pros**:
- Completely offline
- Free (no API costs)
- Full data privacy
- No internet required

**Cons**:
- Slower (5-10 seconds per image)
- Requires local installation
- Needs powerful hardware (recommended: 16GB RAM, GPU)

**Setup**:
```powershell
# Download from https://ollama.ai
# Install and start Ollama
ollama pull llava:13b
```

---

## Implementation

### Script Location
`scripts/task1/process_task1_ai.py` (711 lines)

### Usage Examples

**Basic (Gemini)**:
```powershell
py scripts\task1\process_task1_ai.py --provider gemini --gemini-key YOUR_KEY
```

**Basic (Ollama)**:
```powershell
py scripts\task1\process_task1_ai.py --provider ollama --ollama-model llava:13b
```

**Specific Folder**:
```powershell
py scripts\task1\process_task1_ai.py --provider ollama --folder 20260202_120000
```

**Full Pipeline with Upload**:
```powershell
py scripts\task1\process_task1_ai.py `
  --provider gemini `
  --gemini-key $env:GEMINI_API_KEY `
  --upload-to-jetson `
  --verbose
```

**Custom Configuration**:
```powershell
py scripts\task1\process_task1_ai.py `
  --provider ollama `
  --ollama-model llava:34b `
  --ollama-host http://localhost:11434 `
  --jetson-ip 100.85.121.98 `
  --local-dir C:\NOMAD\Task1 `
  --upload-to-jetson
```

---

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--provider` | (required) | AI provider: `gemini` or `ollama` |
| `--jetson-ip` | `100.85.121.98` | Jetson Tailscale IP |
| `--jetson-port` | `8000` | Edge Core API port |
| `--local-dir` | `C:\NOMAD\Task1` | Local save directory |
| `--gemini-key` | (required if gemini) | Google Gemini API key |
| `--gemini-model` | `gemini-1.5-flash` | Gemini model (flash/pro) |
| `--ollama-model` | `llava:13b` | Ollama vision model |
| `--ollama-host` | `http://localhost:11434` | Ollama server URL |
| `--folder` | (all) | Process specific folder only |
| `--upload-to-jetson` | False | Upload descriptions to Jetson |
| `--verbose` | False | Detailed output |

---

## Jetson API Endpoints

### 1. List Captures
```
GET /api/task/1/captures
```

**Response**:
```json
{
  "captures": ["20260202_120000", "20260202_120130", ...],
  "count": 15
}
```

### 2. Download Files
```
GET /api/task/1/images/{folder}/{filename}
```

**Examples**:
- `/api/task/1/images/20260202_120000/photo.jpg`
- `/api/task/1/images/20260202_120000/metadata.json`

### 3. Upload Description
```
POST /api/task/1/upload_description
```

**Request**:
```json
{
  "folder": "20260202_120000",
  "description": "Scene description text...",
  "provider": "gemini",
  "model": "gemini-1.5-flash"
}
```

**Response**:
```json
{
  "success": true,
  "folder": "20260202_120000",
  "message": "Description uploaded successfully"
}
```

---

## AI Prompt Template

The script generates detailed prompts including:

- GPS coordinates and altitude
- Aircraft heading, pitch, roll
- Building location context
- Specific analysis requests:
  - Scene description
  - Localization features (street signs, building numbers, landmarks)
  - Relative positioning to competition building
  - Competition relevance assessment

**Example Prompt**:
```
Analyze this outdoor reconnaissance photo with the following context:

GPS Location: 45.123456, -75.654321 at 100.5m altitude
Aircraft Heading: 45.0 degrees (0=North, 90=East, 180=South, 270=West)
Aircraft Attitude: Pitch -5.2 degrees, Roll 2.1 degrees
Building Location: Competition Building A (45.123456, -75.654321)

Please provide a detailed description including:

1. SCENE DESCRIPTION:
   - Identify all visible landmarks
   - Describe terrain and environment
   - Note weather and visibility

2. LOCALIZATION FEATURES:
   - Unique identifying features
   - Text visible in image
   - Distinctive architectural features

3. RELATIVE POSITION TO BUILDING:
   - Distance and direction to reference building
   - Viewing angle
   - Line-of-sight assessment

4. COMPETITION RELEVANCE:
   - Image quality assessment
   - Potential target areas
   - Optimal capture angle suggestions
```

---

## Example Output

**description.txt**:
```
SCENE DESCRIPTION:
This photo shows an urban commercial area with a large parking lot. The main
building is a three-story office complex with blue-tinted windows. A McDonald's
restaurant is visible to the right with distinctive golden arches. Approximately
40 vehicles in parking lot with clear lane markings.

LOCALIZATION FEATURES:
- Building address visible: "1234 Commerce Drive"
- Street sign: "Commerce Dr & Tech Blvd"
- Distinctive water tower with "TECH CITY" text
- Cell tower with red aviation lights

RELATIVE POSITION TO BUILDING:
Based on 45-degree heading (northeast facing), Competition Building A is
approximately 150 meters to the southeast. No direct line-of-sight due to
main office building obstruction. Elevated viewing angle at ~35 degrees.

COMPETITION RELEVANCE:
Image quality: EXCELLENT - High clarity, good lighting, minimal blur.
Evidence suitability: SUITABLE - Clear landmarks, readable text.
McDonald's could serve as waypoint for navigation.
Recommend 180-degree heading from this position for direct building view.
```

---

## Dependencies

### Python Libraries
```
requests>=2.31.0      # HTTP client
pillow>=10.1.0        # Image processing
google-generativeai   # Gemini (optional)
```

### External Services
- Gemini API (if using Gemini provider)
- Ollama server (if using Ollama provider)

---

## Performance

### Gemini (gemini-1.5-flash)
- **Speed**: 1-2 seconds per image
- **Throughput**: 30-60 images/minute
- **Cost**: $0.001 per image
- **100 images**: ~2 minutes, ~$0.10

### Ollama (llava:13b on typical hardware)
- **Speed**: 5-10 seconds per image
- **Throughput**: 6-12 images/minute
- **Cost**: Free
- **100 images**: ~10 minutes, $0

### Hardware Requirements (Ollama)
- **Minimum**: Intel Core i5, 8GB RAM
- **Recommended**: Intel Core i7, 16GB RAM, NVIDIA GTX 1650+
- **Optimal**: Intel Core i9, 32GB RAM, NVIDIA RTX 3060+

---

## Security & Privacy

### Gemini (Cloud)
- Images sent to Google servers
- Subject to Google's privacy policy
- May be used for model improvement (check terms)
- Use with caution for sensitive imagery

### Ollama (Local)
- All processing on local Windows machine
- No data leaves ground station
- Fully offline capable
- Recommended for competition/sensitive data

### API Security
- Tailscale VPN for Jetson communication
- Path traversal protection on endpoints
- Filename whitelisting (photo.jpg, metadata.json, description.txt)
- Description upload size limited to 10KB

---

## Troubleshooting

### Cannot Connect to Jetson
```powershell
# Check Tailscale
tailscale status

# Test connectivity
curl http://100.85.121.98:8000/health
```

### Gemini API Errors
```powershell
# Verify API key
python -c "import google.generativeai as genai; genai.configure(api_key='YOUR_KEY'); print('OK')"

# Check quota
# Visit: https://console.cloud.google.com/apis/api/generativelanguage.googleapis.com/quotas
```

### Ollama Not Found
```powershell
# Check Ollama running
Get-Process ollama

# Restart Ollama
ollama serve

# Test API
curl http://localhost:11434/api/version
```

### Poor Description Quality
```powershell
# Try better model (Gemini Pro - higher quality)
--provider gemini --gemini-model gemini-1.5-pro

# Try larger Ollama model
--provider ollama --ollama-model llava:34b
```

---

## Cost Analysis

### Gemini Pricing (as of Feb 2026)
- **Input**: $0.35 per 1M tokens
- **Output**: $1.05 per 1M tokens
- **Typical usage**: ~500 tokens input + 500 tokens output
- **Cost per image**: ~$0.001
- **Competition usage estimate**: 50-100 images = $0.05-0.10

### Ollama Pricing
- **Cost**: Free
- **Hardware**: Already owned ground station laptop

---

## Future Enhancements

- [ ] OpenAI GPT-4 Vision support
- [ ] Anthropic Claude Vision support
- [ ] Batch API optimizations
- [ ] Description quality scoring
- [ ] Automatic model fallback (Ollama → Gemini)
- [ ] Panorama stitching before AI processing
- [ ] JSON-structured output (landmarks, distances, angles)
- [ ] Mission Planner UI integration (one-click process)

---

## Related Documentation

- [Task 1 Photo Capture](task1_photo_capture.md) - Photo capture system
- [../JETSON_DEPLOYMENT.md](../JETSON_DEPLOYMENT.md) - Jetson setup
- [../../scripts/README_AI.md](../../scripts/README_AI.md) - Detailed AI script guide

---

## Quick Reference

### Setup Commands
```powershell
# Gemini
pip install google-generativeai
$env:GEMINI_API_KEY="your-key"

# Ollama
# Download from https://ollama.ai
ollama pull llava:13b
```

### Process All Captures
```powershell
# Gemini (cloud, fast)
py scripts\task1\process_task1_ai.py --provider gemini --gemini-key $env:GEMINI_API_KEY

# Ollama (local, private)
py scripts\task1\process_task1_ai.py --provider ollama
```

### Files Created
- Windows: `C:\NOMAD\Task1\{timestamp}\description.txt`
- Jetson (if uploaded): `data/task1_captures/{timestamp}/description.txt`

---

*NOMAD - McGill Aerial Design AEAC 2026*
