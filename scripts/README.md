# Scripts

Utility scripts for development and operations.

## Development Scripts

### `run_dev.ps1` (Windows)
Run NOMAD Edge Core in simulation mode for local development without hardware.

```powershell
# Default - runs on port 8000
.\scripts\run_dev.ps1

# Custom port
.\scripts\run_dev.ps1 -Port 8080

# Disable vision for API-only testing
.\scripts\run_dev.ps1 -NoVision
```

### `run_dev.sh` (Linux/macOS)
Same as above for Unix systems.

```bash
./scripts/run_dev.sh
./scripts/run_dev.sh --port 8080
./scripts/run_dev.sh --no-vision
```

## Environment
- Sets `NOMAD_SIM_MODE=true` to enable mock hardware
- API available at `http://localhost:8000`
- API docs at `http://localhost:8000/docs`

## AI Processing Scripts

### `process_task1_ai.py`
Process Task 1 captured photos with AI to generate detailed scene descriptions. Supports both cloud (Gemini) and local (Ollama) AI providers. Runs on Windows ground station with automatic download from Jetson.

**Documentation**: See [README_AI.md](README_AI.md) for complete guide.

**Quick Start with Gemini (Cloud)**:
```powershell
python scripts\process_task1_ai.py --provider gemini --gemini-key YOUR_API_KEY
```

**Quick Start with Ollama (Local)**:
```powershell
ollama pull llava:13b
python scripts\process_task1_ai.py --provider ollama --ollama-model llava:13b
```

**Features**:
- Dual AI provider support (Gemini cloud, Ollama local)
- Automatic download from Jetson via HTTP API
- Retry logic for network failures
- Optional upload of descriptions back to Jetson
- Batch processing with progress tracking
- Saves descriptions to `C:\NOMAD\Task1\`

**Requirements**:
```powershell
pip install requests pillow

# For Gemini
pip install google-generativeai

# For Ollama
# Install from https://ollama.ai
ollama pull llava:13b
```

### `process_task1_with_gemini.py` (Legacy)
Original Gemini-only script. Replaced by `process_task1_ai.py` with dual provider support. Kept for backward compatibility.

**Documentation**: See [README_GEMINI.md](README_GEMINI.md)
