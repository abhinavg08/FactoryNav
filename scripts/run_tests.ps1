param(
    [ValidateSet('baseline','factory','dynamic')][string]$Scenario = 'dynamic',
    [ValidateRange(1,10)][int]$Repeat = 2
)
$ErrorActionPreference = 'Stop'
$factorynavRoot = Split-Path $PSScriptRoot -Parent
Push-Location $factorynavRoot
try {
    $composeArgs = @('-f', 'compose.yaml')
    if ($Scenario -eq 'baseline') { $composeArgs += @('-f', 'compose.baseline.yaml') }
    for ($run = 1; $run -le $Repeat; $run++) {
        docker compose @composeArgs up -d --no-build --force-recreate
        if ($LASTEXITCODE -ne 0) { throw 'Simulation restart failed.' }
        $goalArgs = '--x 4.5 --y 0.0 --initial-x -4.5 --initial-y 0.0'
        if ($Scenario -eq 'baseline') { $goalArgs = '--x 0.0 --y 1.7' }
        if ($Scenario -eq 'dynamic') { $goalArgs += ' --obstacle-after-m 1.0' }
        $goalCommand = "source /opt/ros/jazzy/setup.bash; python3 /workspace/scripts/send_goal.py $goalArgs --timeout 240 --output /workspace/log/test_${Scenario}_${run}.json"
        docker compose @composeArgs exec -T simulation bash -lc $goalCommand
        if ($LASTEXITCODE -ne 0) { throw "Scenario $Scenario run $run failed; inspect log/." }
    }
} finally {
    Pop-Location
}
