# Contributing to FactoryNav

Keep changes scoped and describe the behaviour they affect. The [setup guide](docs/setup.md) explains the Docker runtime and separate Windows tooling environment.

## Lightweight verification

```powershell
.\.conda\python.exe scripts/check_repository.py --history
$env:PYTHONPATH = Join-Path $PWD 'factorynav_dynamic'
.\.conda\python.exe -m unittest discover -s factorynav_dynamic/test -v
```

For a native critic change, rebuild in the project container and run its CTest suite. For a change that affects navigation, run the relevant fresh-world scenarios from the [command reference](docs/dynamic_commands.md). Documentation and presentation changes do not need a lengthy simulation matrix.

## Evidence and reproducibility

- Keep the original factory and its regressions working. The original implementation is preserved on the `baseline/original-factory` branch and in Git history.
- Never overwrite completed evidence or rewrite recorded source hashes. Use a new output directory and report failed acceptance checks alongside successful goals.
- Keep ground truth out of perception and control. Actors must remain independent of robot state.
- Clearly distinguish YOLO recognition, LiDAR geometry, navigation control and offline evaluation.
- Do not commit credentials, downloaded weights/assets, environments, build products, private machine configuration or recordings. New experiment output is ignored until deliberately promoted as evidence.
- Respect third-party licenses and retain model attribution. The root MIT license covers original project code, not upstream dependencies.
