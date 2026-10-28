# Dot-source this script: . .\setup\activate.ps1
$factorynavCondaRoot = 'D:\ana3'
$factorynavEnvPath = Join-Path (Split-Path $PSScriptRoot -Parent) '.conda'
if (-not (Test-Path -LiteralPath (Join-Path $factorynavEnvPath 'python.exe'))) {
    throw "Environment missing at $factorynavEnvPath. Run setup/create_env.ps1 first."
}
if (-not (Get-Command conda -ErrorAction SilentlyContinue)) {
    . (Join-Path $factorynavCondaRoot 'shell\condabin\conda-hook.ps1')
}
conda activate $factorynavEnvPath
if ($LASTEXITCODE -ne 0) { throw 'Conda activation failed.' }
