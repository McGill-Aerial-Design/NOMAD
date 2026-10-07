# SPDX-License-Identifier: Apache-2.0
[CmdletBinding()]
param(
    [ValidateSet('Plan', 'Protect', 'Install', 'Uninstall')][string]$Action = 'Plan',
    [string]$Executable,
    [string]$Config
)
$ErrorActionPreference = 'Stop'

function Test-AbsolutePath {
    param([string]$Path)
    if ([string]::IsNullOrWhiteSpace($Path) -or $Path -match '["\r\n]') {
        return $false
    }
    try {
        # Rooted drive-relative paths still depend on the caller's working directory.
        return [IO.Path]::IsPathRooted($Path) -and
            ([IO.Path]::GetPathRoot($Path) -eq [IO.Path]::GetPathRoot([IO.Path]::GetFullPath($Path)))
    } catch {
        return $false
    }
}

function Get-ServiceCommand {
    param([string]$Binary, [string]$Configuration)
    foreach ($value in @($Binary, $Configuration)) {
        if (-not (Test-AbsolutePath $value)) {
            throw 'Executable and config must be absolute paths without quotes or line breaks.'
        }
    }
    return ('"{0}" --service --config "{1}"' -f $Binary, $Configuration)
}

function Invoke-ServiceTool {
    param([string[]]$ToolArguments)
    & "$env:SystemRoot/System32/sc.exe" @ToolArguments
    if ($LASTEXITCODE -ne 0) { throw 'Service configuration failed.' }
}

function Set-PrivatePath {
    param([string]$Path, [bool]$Directory)
    $sid = [Security.Principal.SecurityIdentifier]::new('S-1-5-19')
    if ($Directory) {
        $acl = [Security.AccessControl.DirectorySecurity]::new()
        $inherit = [Security.AccessControl.InheritanceFlags]'ContainerInherit, ObjectInherit'
    } else {
        $acl = [Security.AccessControl.FileSecurity]::new()
        $inherit = [Security.AccessControl.InheritanceFlags]::None
    }
    $acl.SetOwner($sid)
    $acl.SetAccessRuleProtection($true, $false)
    foreach ($identity in @('S-1-5-19', 'S-1-5-18', 'S-1-5-32-544')) {
        $rule = [Security.AccessControl.FileSystemAccessRule]::new(
            [Security.Principal.SecurityIdentifier]::new($identity), 'FullControl', $inherit,
            [Security.AccessControl.PropagationFlags]::None, 'Allow')
        $acl.AddAccessRule($rule)
    }
    Set-Acl -LiteralPath $Path -AclObject $acl
}

function Test-ProvisionedPath {
    param([string]$Path, [bool]$Directory)
    if (-not (Test-AbsolutePath $Path)) {
        throw 'Provision absolute configuration paths without quotes or line breaks.'
    }
    $item = Get-Item -LiteralPath $Path -Force
    if ($item.PSIsContainer -ne $Directory) { throw 'Unexpected configuration path type.' }
    while ($null -ne $item) {
        if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {
            throw 'Reparse points are not supported, including parent directories.'
        }
        $item = if ($item.PSIsContainer) { $item.Parent } else { $item.Directory }
    }
}

if ($Action -eq 'Uninstall') {
    $service = Get-Service nomad-runtime -ErrorAction SilentlyContinue
    if ($service) {
        Stop-Service nomad-runtime
        $service.WaitForStatus('Stopped', [TimeSpan]::FromSeconds(30))
        Invoke-ServiceTool @('delete', 'nomad-runtime')
    }
    return
}
$command = Get-ServiceCommand $Executable $Config
if ($Action -eq 'Plan') {
    [pscustomobject]@{ BinaryPath = $command; Account = 'NT AUTHORITY\LocalService'; Start = 'demand';
        Recovery = 'restart/5000/restart/30000/none/0'; ResetSeconds = 86400; NonCrashRecovery = $false }
    return
}
if ($Action -eq 'Protect') {
    Test-ProvisionedPath $Config $false
    $settings = Get-Content -LiteralPath $Config -Raw | ConvertFrom-Json
    $credentials = $settings.NOMAD_CLIENT_CREDENTIALS_FILE
    $audit = $settings.NOMAD_AUDIT_DIRECTORY
    Test-ProvisionedPath $credentials $false
    if (-not (Test-AbsolutePath $audit)) { throw 'Provision an absolute audit directory.' }
    if (-not (Test-Path -LiteralPath $audit)) {
        Test-ProvisionedPath (Split-Path -Parent $audit) $true
    }
    $actuators = $settings.NOMAD_ACTUATORS_FILE
    if (-not [string]::IsNullOrEmpty($actuators)) {
        Test-ProvisionedPath $actuators $false
        $actuatorDirectory = Split-Path -Parent ([IO.Path]::GetFullPath($actuators))
        $entries = @(Get-ChildItem -LiteralPath $actuatorDirectory -Force)
        if ($entries.Count -ne 1 -or $entries[0].FullName -ne [IO.Path]::GetFullPath($actuators)) {
            throw 'Use a dedicated actuator directory containing only the reviewed actuator file.'
        }
    }
    if (-not (Test-Path -LiteralPath $audit)) { New-Item -ItemType Directory -Path $audit | Out-Null }
    Test-ProvisionedPath $audit $true
    Set-PrivatePath $Config $false
    Set-PrivatePath $credentials $false
    Set-PrivatePath $audit $true
    if (-not [string]::IsNullOrEmpty($actuators)) {
        Set-PrivatePath $actuatorDirectory $true
        Set-PrivatePath $actuators $false
    }
    return
}
if (-not (Test-Path -LiteralPath $Executable -PathType Leaf) -or
    -not (Test-Path -LiteralPath $Config -PathType Leaf)) { throw 'Provision executable and config first.' }
if (-not [Diagnostics.EventLog]::SourceExists('nomad-runtime')) {
    [Diagnostics.EventLog]::CreateEventSource('nomad-runtime', 'Application')
}
if (Get-Service nomad-runtime -ErrorAction SilentlyContinue) {
    $service = Get-Service nomad-runtime
    if ($service.Status -ne 'Stopped') { throw 'Stop the runtime before upgrading service configuration.' }
    $installed = Get-CimInstance -ClassName Win32_Service -Filter "Name='nomad-runtime'"
    $result = Invoke-CimMethod -InputObject $installed -MethodName Change -Arguments @{
        PathName = $command; StartMode = 'Manual'; StartName = 'NT AUTHORITY\LocalService'; StartPassword = ''
    }
} else {
    $result = Invoke-CimMethod -ClassName Win32_Service -MethodName Create -Arguments @{
        Name = 'nomad-runtime'; DisplayName = 'NOMAD runtime'; PathName = $command
        ServiceType = [byte]16; ErrorControl = [byte]1; StartMode = 'Manual'; DesktopInteract = $false
        StartName = 'NT AUTHORITY\LocalService'; StartPassword = ''
    }
}
if ($result.ReturnValue -ne 0) { throw "SCM registration failed with code $($result.ReturnValue)." }
Invoke-ServiceTool @('failure', 'nomad-runtime', 'reset=', '86400', 'actions=', 'restart/5000/restart/30000/none/0')
Invoke-ServiceTool @('failureflag', 'nomad-runtime', '0')
