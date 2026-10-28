param([string]$CondaExe = 'D:\ana3\Scripts\conda.exe')
$ErrorActionPreference = 'Stop'
$factorynavRoot = Split-Path $PSScriptRoot -Parent
$factorynavEnvPath = Join-Path $factorynavRoot '.conda'
if (Test-Path -LiteralPath (Join-Path $factorynavEnvPath 'conda-meta\history')) {
    Write-Host "Existing environment: $factorynavEnvPath"
    exit 0
}
if (-not (Test-Path -LiteralPath $CondaExe)) {
    throw "Conda not found at $CondaExe. Pass -CondaExe with the installed conda.exe path."
}
& $CondaExe env create --prefix $factorynavEnvPath --file (Join-Path $factorynavRoot 'environment.yml') --yes
if ($LASTEXITCODE -ne 0) { throw 'Conda environment creation failed.' }
Write-Host 'Activate in PowerShell with: . .\setup\activate.ps1'
