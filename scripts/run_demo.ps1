param([switch]$Build, [switch]$Baseline, [switch]$Perception,
      [switch]$DynamicFactory, [switch]$Headless,
      [switch]$DebugTrajectories,
      [ValidateSet('baseline','dynamic')][string]$ControllerProfile='dynamic',
      [ValidateSet('none','full')][string]$Traffic='full',
      [double]$StartX=-8, [double]$StartY=-6.5, [double]$StartYaw=0)
$ErrorActionPreference = 'Stop'
$factorynavRoot = Split-Path $PSScriptRoot -Parent
Push-Location $factorynavRoot
try {
    if ($Baseline -and $Perception) { throw 'Perception is supported in the factory scenario.' }
    if ($Baseline -and $DynamicFactory) { throw 'Choose one world mode.' }
    $composeArgs = @('-f', 'compose.yaml')
    if ($Baseline) { $composeArgs += @('-f', 'compose.baseline.yaml') }
    if ($Perception) { $composeArgs += @('-f', 'compose.perception.yaml') }
    if ($DynamicFactory) {
        $composeArgs += @('-f','compose.dynamic.yaml')
        $env:FACTORYNAV_PROFILE=$ControllerProfile
        $env:FACTORYNAV_GUI=(!$Headless).ToString().ToLowerInvariant()
        $env:FACTORYNAV_PERCEPTION=$Perception.ToString().ToLowerInvariant()
        $env:FACTORYNAV_TRAFFIC=$Traffic
        $env:FACTORYNAV_DEBUG_TRAJECTORIES=$DebugTrajectories.ToString().ToLowerInvariant()
        $env:FACTORYNAV_START_X=$StartX.ToString([cultureinfo]::InvariantCulture)
        $env:FACTORYNAV_START_Y=$StartY.ToString([cultureinfo]::InvariantCulture)
        $env:FACTORYNAV_START_YAW=$StartYaw.ToString([cultureinfo]::InvariantCulture)
    }
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
    if (!$Headless) { Write-Host 'Gazebo + RViz desktop: http://localhost:6080/vnc.html?autoconnect=1&resize=scale' }
    Write-Host 'Logs: docker compose logs -f simulation'
    Write-Host 'Stop: docker compose down'
} finally {
    Pop-Location
}
