#!/usr/bin/env pwsh
# Setup SSH key authentication to Jetson Orin Nano
# Run this script once: .\scripts\setup_ssh_jetson.ps1

$JetsonIP = "100.85.121.98"
$JetsonUser = "mad"
$PubKeyPath = "$env:USERPROFILE\.ssh\id_rsa.pub"

if (-not (Test-Path $PubKeyPath)) {
    Write-Host "No SSH key found. Generating one..."
    ssh-keygen -t rsa -b 4096 -f "$env:USERPROFILE\.ssh\id_rsa" -N '""'
}

$pubKey = Get-Content $PubKeyPath -Raw
$pubKey = $pubKey.Trim()

Write-Host "Copying SSH key to $JetsonUser@$JetsonIP..."
Write-Host "You will be prompted for the password ONE TIME."
Write-Host ""

# Use ssh to create the authorized_keys file
ssh ${JetsonUser}@${JetsonIP} "mkdir -p ~/.ssh && echo '$pubKey' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && chmod 700 ~/.ssh && echo 'SSH KEY INSTALLED SUCCESSFULLY'"

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "Testing passwordless connection..."
    ssh ${JetsonUser}@${JetsonIP} "echo 'PASSWORDLESS SSH WORKS - Connection verified'"
} else {
    Write-Host "Failed to copy SSH key. Please check the password and try again."
}
