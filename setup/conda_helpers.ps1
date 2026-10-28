# Locate Conda without assuming a particular drive or user account.
function Resolve-FactoryNavConda {
    param([string]$CondaExe)
    if ($CondaExe) {
        if (-not (Test-Path -LiteralPath $CondaExe -PathType Leaf)) {
            throw "Conda not found at $CondaExe."
        }
        return (Resolve-Path -LiteralPath $CondaExe).Path
    }
    $factorynavCandidates = @($env:CONDA_EXE)
    $factorynavCommand = Get-Command conda.exe -CommandType Application -ErrorAction SilentlyContinue
    if ($factorynavCommand) { $factorynavCandidates += $factorynavCommand.Source }
    if (Get-Command conda -CommandType Function -ErrorAction SilentlyContinue) {
        $factorynavDetectedBase = conda info --base
        if ($LASTEXITCODE -eq 0 -and $factorynavDetectedBase) {
            $factorynavCandidates += Join-Path $factorynavDetectedBase 'Scripts\conda.exe'
        }
    }
    foreach ($factorynavParent in @($env:USERPROFILE, $env:LOCALAPPDATA, $env:ProgramData)) {
        if (-not $factorynavParent) { continue }
        foreach ($factorynavDistribution in @('miniconda3', 'anaconda3', 'miniforge3', 'mambaforge')) {
            $factorynavCandidates += Join-Path $factorynavParent "$factorynavDistribution\Scripts\conda.exe"
        }
    }
    foreach ($factorynavCandidate in $factorynavCandidates) {
        if ($factorynavCandidate -and (Test-Path -LiteralPath $factorynavCandidate -PathType Leaf)) {
            return (Resolve-Path -LiteralPath $factorynavCandidate).Path
        }
    }
    throw 'Conda was not found. Open a Conda PowerShell prompt or pass -CondaExe with your conda.exe path.'
}
