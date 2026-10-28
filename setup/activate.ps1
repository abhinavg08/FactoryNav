# Dot-source this script: . .\setup\activate.ps1 [-CondaExe <path-to-conda.exe>]
param([string]$CondaExe)
$factorynavEnvPath = Join-Path (Split-Path $PSScriptRoot -Parent) '.conda'
if (-not (Test-Path -LiteralPath (Join-Path $factorynavEnvPath 'python.exe'))) {
    throw "Environment missing at $factorynavEnvPath. Run setup/create_env.ps1 first."
}
if (-not (Get-Command conda -CommandType Function -ErrorAction SilentlyContinue)) {
    . (Join-Path $PSScriptRoot 'conda_helpers.ps1')
    $factorynavCondaExe = Resolve-FactoryNavConda -CondaExe $CondaExe
    $factorynavCondaRoot = Split-Path (Split-Path $factorynavCondaExe -Parent) -Parent
    $factorynavCondaHook = Join-Path $factorynavCondaRoot 'shell\condabin\conda-hook.ps1'
    if (-not (Test-Path -LiteralPath $factorynavCondaHook)) {
        throw "Conda PowerShell hook missing: $factorynavCondaHook. Use a Conda PowerShell prompt."
    }
    . $factorynavCondaHook
}
conda activate $factorynavEnvPath
if ($LASTEXITCODE -ne 0) { throw 'Conda activation failed.' }
