param([switch]$Build, [switch]$Baseline, [switch]$Perception)
$ErrorActionPreference = 'Stop'
$factorynavRoot = Split-Path $PSScriptRoot -Parent
Push-Location $factorynavRoot
try {
    if ($Baseline -and $Perception) { throw 'Perception is supported in the factory scenario.' }
    $composeArgs = @('-f', 'compose.yaml')
    if ($Baseline) { $composeArgs += @('-f', 'compose.baseline.yaml') }
    if ($Perception) { $composeArgs += @('-f', 'compose.perception.yaml') }
    if ($Build) {
        docker compose -f compose.yaml build
        if ($LASTEXITCODE -ne 0) { throw 'Base image build failed.' }
        if ($Perception) {
            docker run --rm --mount "type=bind,source=$factorynavRoot,target=/workspace" factorynav-ai:jazzy python3 /workspace/setup/cache_perception_wheels.py
            if ($LASTEXITCODE -ne 0) { throw 'Perception wheel caching failed.' }
            docker compose @composeArgs build
            if ($LASTEXITCODE -ne 0) { throw 'Perception image build failed.' }
        }
    }
    docker compose @composeArgs up -d --no-build
    if ($LASTEXITCODE -ne 0) { throw 'Simulation launch failed.' }
    Write-Host 'Gazebo + RViz desktop: http://localhost:6080/vnc.html?autoconnect=1&resize=scale'
    Write-Host 'Logs: docker compose logs -f simulation'
    Write-Host 'Stop: docker compose down'
} finally {
    Pop-Location
}
