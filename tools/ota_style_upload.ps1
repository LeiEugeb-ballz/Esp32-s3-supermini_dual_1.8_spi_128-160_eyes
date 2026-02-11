param(
  [ValidateSet("1", "2", "3", "4", "5")]
  [string]$Style = "1",
  [Parameter(Mandatory = $true)]
  [string]$Device,
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$envMap = @{
  "1" = "esp32s3-eyes"
  "2" = "esp32s3-eyes-2"
  "3" = "esp32s3-eyes-3"
  "4" = "esp32s3-eyes-4"
  "5" = "esp32s3-eyes-5"
}

$envName = $envMap[$Style]
if (-not $envName) {
  throw "Unsupported style '$Style'. Use 1..5."
}

if (-not $SkipBuild) {
  Write-Host "Building style $Style using env '$envName'..."
  $pioCmd = Get-Command pio -ErrorAction SilentlyContinue
  if (-not $pioCmd) {
    $pioFallback = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
    if (Test-Path $pioFallback) {
      $pioCmd = Get-Command $pioFallback
    }
  }
  if (-not $pioCmd) {
    throw "PlatformIO CLI not found. Install PlatformIO or add 'pio' to PATH."
  }

  & $pioCmd.Source run -e $envName
  if ($LASTEXITCODE -ne 0) {
    throw "Build failed for env '$envName'."
  }
}

$binPath = Join-Path $repoRoot ".pio\build\$envName\firmware.bin"
if (-not (Test-Path $binPath)) {
  throw "Firmware binary not found: $binPath"
}

$url = "http://$Device/update"
Write-Host "Uploading '$binPath' to $url ..."

$curl = Get-Command curl.exe -ErrorAction SilentlyContinue
if (-not $curl) {
  throw "curl.exe was not found in PATH."
}

& curl.exe -sS -X POST -F "firmware=@$binPath" $url
if ($LASTEXITCODE -ne 0) {
  throw "OTA upload failed."
}

Write-Host "Done. Device should reboot if update succeeded."
