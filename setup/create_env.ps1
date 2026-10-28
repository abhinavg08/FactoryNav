param([string]$CondaExe)
$ErrorActionPreference = 'Stop'
$factorynavRoot = Split-Path $PSScriptRoot -Parent
$factorynavEnvPath = Join-Path $factorynavRoot '.conda'
if (Test-Path -LiteralPath (Join-Path $factorynavEnvPath 'conda-meta\history')) {
    Write-Host "Existing environment: $factorynavEnvPath"
    return
}
. (Join-Path $PSScriptRoot 'conda_helpers.ps1')
$factorynavCondaExe = Resolve-FactoryNavConda -CondaExe $CondaExe
& $factorynavCondaExe env create --prefix $factorynavEnvPath --file (Join-Path $factorynavRoot 'environment.yml') --yes
if ($LASTEXITCODE -ne 0) { throw 'Conda environment creation failed.' }
Write-Host 'Activate in PowerShell with: . .\setup\activate.ps1'
