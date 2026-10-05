# SPDX-License-Identifier: Apache-2.0
param([string]$Helper, [string]$Config)
$ErrorActionPreference = 'Stop'
$global:provisionChanges = @()
function Set-Acl {
    param($LiteralPath, $AclObject)
    $rules = @($AclObject.GetAccessRules($true, $true, [Security.Principal.SecurityIdentifier]) | ForEach-Object {
        @{ SID = $_.IdentityReference.Value; Rights = [string]$_.FileSystemRights;
           Inheritance = [string]$_.InheritanceFlags }
    })
    $owner = $AclObject.GetOwner([Security.Principal.SecurityIdentifier]).Value
    $global:provisionChanges += @{ Path = $LiteralPath; Owner = $owner;
                         Protected = $AclObject.AreAccessRulesProtected; Rules = $rules }
}
& $Helper -Action Protect -Executable "$env:SystemRoot\System32\notepad.exe" -Config $Config
ConvertTo-Json -InputObject $global:provisionChanges -Depth 5 -Compress
