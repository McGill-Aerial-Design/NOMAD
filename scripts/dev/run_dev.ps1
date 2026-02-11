# =============================================================================
# NOMAD Development Runner (Windows PowerShell)
# =============================================================================
# Convenience script to run NOMAD Edge Core in simulation mode for development.
# 
# This enables testing the full system (Web UI, Telemetry, Vision) on a laptop
# without requiring actual hardware (ZED camera, Jetson, ArduPilot).
#
# Usage:
#   .\scripts\run_dev.ps1             # Run with default settings
#   .\scripts\run_dev.ps1 -Port 8080  # Custom port
#   .\scripts\run_dev.ps1 -NoVision   # Disable vision process
# =============================================================================

param(
    [int]$Port = 8000,
    [string]$BindHost = "0.0.0.0",
    [string]$LogLevel = "info",
    [switch]$NoVision,
    [switch]$NoTask2,
    [string]$ServoMode = "gimbal"
)

# Get script directory and project root
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

Write-Host "=============================================" -ForegroundColor Blue
Write-Host "  NOMAD Edge Core - Development Mode" -ForegroundColor Blue
Write-Host "=============================================" -ForegroundColor Blue
Write-Host ""

# Find Python
$PythonCmd = $null
if (Get-Command "py" -ErrorAction SilentlyContinue) {
    $PythonCmd = "py"
} elseif (Get-Command "python" -ErrorAction SilentlyContinue) {
    $PythonCmd = "python"
} else {
    Write-Host "Error: Python not found. Please install Python 3.13+" -ForegroundColor Red
    exit 1
}

# Get Python version
$PyVersion = & $PythonCmd -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
Write-Host "Using Python $PyVersion" -ForegroundColor Green

# Change to project directory
Push-Location $ProjectRoot

try {
    # Set PYTHONPATH
    $env:PYTHONPATH = "$ProjectRoot;$env:PYTHONPATH"
    Write-Host "PYTHONPATH set to: $ProjectRoot" -ForegroundColor Green

    # Enable simulation mode
    $env:NOMAD_SIM_MODE = "true"
    Write-Host "NOMAD_SIM_MODE=true (mock hardware enabled)" -ForegroundColor Yellow

    # Additional environment variables for development
    $env:VISION_VIDEO_SOURCE = "zed"
    $env:ENABLE_VIO = "true"
    $env:VISION_CONFIDENCE = "0.5"

    Write-Host ""
    Write-Host "Starting Edge Core server..." -ForegroundColor Blue
    Write-Host "API will be available at: http://localhost:$Port" -ForegroundColor Green
    Write-Host "API docs at: http://localhost:$Port/docs" -ForegroundColor Green
    Write-Host ""

    # Build arguments
    $CmdArgs = @("--sim", "--host", $BindHost, "--port", $Port, "--log-level", $LogLevel)
    
    if ($NoVision) {
        $CmdArgs += "--no-vision"
    }
    
    if ($NoTask2) {
        $CmdArgs += "--no-task2"
    }
    
    $CmdArgs += @("--servo-mode", $ServoMode)

    # Run the Edge Core with simulation mode
    & $PythonCmd -m edge_core.main @CmdArgs

} finally {
    Pop-Location
}
