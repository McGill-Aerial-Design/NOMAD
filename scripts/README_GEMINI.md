# Task 1 Gemini Integration Script

## Overview

Python script to process Task 1 captured photos using Google Gemini AI for automated scene description and localization analysis.

## Installation

```bash
# On Jetson or Windows
pip3 install google-generativeai pillow
```

## Configuration

### Get Gemini API Key
1. Visit: https://makersuite.google.com/app/apikey
2. Create new API key
3. Store securely (do NOT commit to git)

### Set Environment Variable (Recommended)
```bash
# Linux/macOS
export GEMINI_API_KEY="your-api-key-here"

# Windows PowerShell
$env:GEMINI_API_KEY="your-api-key-here"
```

## Usage

### Batch Process All Captures
```bash
python scripts/process_task1_with_gemini.py --gemini-key $GEMINI_API_KEY
```

### Process Single Folder
```bash
python scripts/process_task1_with_gemini.py \
  --gemini-key $GEMINI_API_KEY \
  --folder 20260202_120000
```

### Use Pro Model (Better Quality)
```bash
python scripts/process_task1_with_gemini.py \
  --gemini-key $GEMINI_API_KEY \
  --model gemini-1.5-pro
```

### Verbose Output
```bash
python scripts/process_task1_with_gemini.py \
  --gemini-key $GEMINI_API_KEY \
  --verbose
```

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--gemini-key` | (required) | Google Gemini API key |
| `--captures-dir` | `data/task1_captures` | Path to captures directory |
| `--folder` | (all folders) | Process only specific folder |
| `--model` | `gemini-1.5-flash` | Model: `gemini-1.5-flash` or `gemini-1.5-pro` |
| `--verbose` | False | Print descriptions and detailed output |

## Output

The script generates `description.txt` in each capture folder:

```
data/task1_captures/
├── 20260202_120000/
│   ├── photo.jpg
│   ├── metadata.json
│   └── description.txt         (GENERATED)
├── 20260202_120130/
│   ├── photo.jpg
│   ├── metadata.json
│   └── description.txt         (GENERATED)
```

### Example Description Output

```
SCENE DESCRIPTION:
This photo shows an urban commercial area with a large parking lot in the foreground. 
The main building is a three-story office complex with blue-tinted windows and 
concrete facade. A McDonald's restaurant is visible to the right, with its distinctive 
golden arches. The parking lot has approximately 40 vehicles and clear lane markings.

LOCALIZATION FEATURES:
- Building address visible: "1234 Commerce Drive"
- McDonald's sign with visible drive-through
- Street sign at intersection: "Commerce Dr & Tech Blvd"
- Distinctive water tower in background with "TECH CITY" text
- Cell tower with red aviation lights on roof of main building

RELATIVE POSITION TO BUILDING:
Based on the 45-degree heading (northeast facing), the reference building "Competition 
Building A" at coordinates (45.123456, -75.654321) is approximately 150 meters to the 
southeast. No direct line-of-sight due to the main office building obstruction. 
The viewing angle is elevated at approximately 35 degrees above horizon, consistent 
with a drone altitude of 100 meters.

COMPETITION RELEVANCE:
Image quality: EXCELLENT - High clarity, good lighting, minimal motion blur.
Evidence suitability: SUITABLE - Clear landmarks and readable text for verification.
Points of interest: McDonald's could serve as waypoint for competition navigation.
Optimal capture angles: For building verification, recommend 180-degree heading from 
this position to get direct view of Competition Building A.
```

## Prompting Strategy

The script generates detailed prompts including:
- GPS coordinates and altitude
- Aircraft heading, pitch, roll
- Building location context
- Specific analysis requests for:
  - Scene description
  - Localization features
  - Relative positioning
  - Competition relevance

## Cost Estimation

### Gemini 1.5 Flash (Recommended)
- **Cost**: $0.35 per 1M tokens (input), $1.05 per 1M tokens (output)
- **Typical usage**: ~500 tokens input + 500 tokens output per image
- **Estimated cost**: $0.001 per image (0.1 cents)
- **100 images**: ~$0.10

### Gemini 1.5 Pro (Higher Quality)
- **Cost**: $3.50 per 1M tokens (input), $10.50 per 1M tokens (output)
- **Typical usage**: ~500 tokens input + 500 tokens output per image
- **Estimated cost**: $0.007 per image (0.7 cents)
- **100 images**: ~$0.70

Source: https://ai.google.dev/pricing (as of Feb 2026)

## Error Handling

The script handles:
- Missing metadata.json files (warning, skip)
- Missing photo.jpg files (error, skip)
- API failures (error, continue to next)
- Already processed folders (skip)
- Network timeouts (retry recommended manually)

## Automation

### Systemd Service (Linux/Jetson)
Create `/etc/systemd/system/task1-gemini-processor.service`:

```ini
[Unit]
Description=Task 1 Gemini AI Processor
After=network.target

[Service]
Type=oneshot
User=mad
WorkingDirectory=/home/mad/NOMAD
Environment="GEMINI_API_KEY=your-key-here"
ExecStart=/usr/bin/python3 scripts/process_task1_with_gemini.py --gemini-key $GEMINI_API_KEY
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

### Cron Job (Periodic Processing)
```bash
# Process new captures every 5 minutes
*/5 * * * * cd /home/mad/NOMAD && python3 scripts/process_task1_with_gemini.py --gemini-key $GEMINI_API_KEY >> /var/log/task1-gemini.log 2>&1
```

### Windows Task Scheduler
```powershell
# Create scheduled task
$action = New-ScheduledTaskAction -Execute "python" -Argument "scripts\process_task1_with_gemini.py --gemini-key YOUR_KEY"
$trigger = New-ScheduledTaskTrigger -Once -At (Get-Date) -RepetitionInterval (New-TimeSpan -Minutes 5)
Register-ScheduledTask -TaskName "Task1GeminiProcessor" -Action $action -Trigger $trigger
```

## Security Notes

- **API Key**: Store securely, do NOT commit to git
- **Rate Limits**: Gemini has generous free tier (15 RPM, 1M tokens/min)
- **Cost Control**: Use `gemini-1.5-flash` for cost efficiency
- **Data Privacy**: Images processed via Gemini API (check Google's data handling policy)

## Troubleshooting

### Import Error: google.generativeai
```bash
pip3 install google-generativeai
```

### Import Error: PIL
```bash
pip3 install pillow
```

### API Key Error
```bash
# Verify API key is valid
echo $GEMINI_API_KEY

# Test with simple request
python3 -c "import google.generativeai as genai; genai.configure(api_key='YOUR_KEY'); print('OK')"
```

### Network Timeout
- Check internet connectivity
- Verify Gemini API status: https://status.cloud.google.com/
- Increase timeout in script if needed

## Future Enhancements

- [ ] Add retry logic for API failures
- [ ] Implement rate limiting to avoid quota exhaustion
- [ ] Add support for batch API calls (more efficient)
- [ ] Generate summary CSV of all descriptions
- [ ] Add OCR for text extraction from signs
- [ ] Implement object detection annotation overlay

## References

- Google Gemini API Docs: https://ai.google.dev/docs
- Pricing: https://ai.google.dev/pricing
- Vision Capabilities: https://ai.google.dev/docs/vision

---

*NOMAD - McGill Aerial Design AEAC 2026*
