# Setup and reproducibility

All active commands run from the root of your cloned `factorynav` directory. The checkout can live on any drive, including a path containing spaces. Docker mounts it at `/workspace`; that fixed container path is intentional and independent of the Windows path.

## Prerequisites

Use Git, Windows PowerShell and Docker Desktop with WSL2/Linux containers. Install Conda for the project-local Windows tooling environment. ROS 2 Jazzy and Gazebo Harmonic run in the supplied Ubuntu 24.04 image. Linux ROS binaries do not run in the Windows Conda environment.

The tested host had 16 GB RAM. The ROS images, optional perception packages and caches need substantial disk space and internet access for the first build. There is no measured fresh-install disk minimum. Rendering uses Mesa llvmpipe and inference uses CPU; an NVIDIA GPU is not required by the validated fallback.

## Clone and create the host environment

```powershell
git clone https://github.com/abhinavg08/factorynav.git
cd factorynav
.\setup\create_env.ps1
. .\setup\activate.ps1
python -c "import sys, numpy, yaml; print(sys.executable); print(numpy.__version__, yaml.__version__)"
python -m pip check
```

The scripts use their own location to resolve the repository root, rather than the current drive or username. They discover Conda from `CONDA_EXE`, an active Conda function, PATH and common Miniconda/Anaconda/Miniforge directories. For a custom installation, supply your actual `conda.exe` path with `-CondaExe` to both scripts. Creation reuses an existing `.conda`; activation changes only the current shell and does not update Conda base or your PowerShell profile.

[`environment.yml`](../environment.yml) specifies Python 3.12, NumPy 1.26.4 and PyYAML 6.0. The original Windows package set is also preserved in [`conda-win-64.lock.txt`](../conda-win-64.lock.txt). From a Conda-enabled PowerShell prompt, use this instead of environment creation to reproduce that Windows x86-64 package set exactly:

```powershell
conda create --prefix .\.conda --file .\conda-win-64.lock.txt --yes
. .\setup\activate.ps1
```

Do not use that Windows lock file for Linux.

## Build the original factory

Start Docker Desktop, select Linux containers, and wait until `docker info` succeeds.

```powershell
.\scripts\run_demo.ps1 -Build
```

The base Dockerfile pins its upstream ROS image by digest, then installs Jazzy navigation, Gazebo integration, RViz and the virtual desktop. Apt package versions are not frozen; the [environment audit](environment_audit.md), [installed package snapshot](ubuntu-packages.lock.txt) and experiment parameter/source snapshots describe the tested environment. A fresh build with later apt packages may need revalidation.

Open the [local Gazebo/RViz desktop](http://localhost:6080/vnc.html?autoconnect=1&resize=scale). The port binds to loopback only. Wait for Navigation and Localization to become active. Send a goal with RViz's **Nav2 Goal** tool or run:

```powershell
docker compose exec -T simulation bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /workspace/scripts/send_goal.py --x 4.5 --y 0 --initial-x -4.5 --initial-y 0'
```

Resetting the container disconnects noVNC; refresh and reconnect after startup. Do not submit manual goals during an automated benchmark.

## Build the advanced factory with optional YOLO

```powershell
# Fetch pinned CC-BY-3.0 model files to the ignored local cache.
.\.conda\python.exe setup/fetch_person.py
docker compose down
.\scripts\run_demo.ps1 -DynamicFactory -Perception -Build
```

The optional build caches pinned wheels from official PyPI, checks their SHA256 values, installs CPU PyTorch in a separate Linux virtual environment and runs `pip check`. Manifests are under [`setup/`](../setup/); the tested installed package list is [`perception-python.lock.txt`](perception-python.lock.txt). The detector obtains its YOLO26 weights on first use; the recorded model source/hash is in [`yolo26_weights.lock.json`](../setup/yolo26_weights.lock.json).

The advanced launcher builds the native MPPI critic in Release mode when needed, then starts the complex factory, motion estimator and selected traffic. The first native build took about 20 s on the tested host. Omit `-Perception` for LiDAR navigation without YOLO; omit `-Build` after the images exist. Use `-Traffic none`, `-Headless` or `-ControllerProfile baseline` as described in the [command reference](dynamic_commands.md).

To build the advanced scene without perception from a fresh checkout:

```powershell
.\.conda\python.exe setup/fetch_person.py
.\scripts\run_demo.ps1 -DynamicFactory -Build
```

## Checks and scenarios

These lightweight checks do not start Gazebo:

```powershell
.\.conda\python.exe scripts/check_repository.py --history
$env:PYTHONPATH = Join-Path $PWD 'factorynav_dynamic'
.\.conda\python.exe -m unittest discover -s factorynav_dynamic/test -v
```

These commands recreate this project's world and run actual navigation:

```powershell
.\scripts\run_tests.ps1 -Scenario factory -Repeat 1
.\scripts\run_tests.ps1 -Scenario dynamic -Repeat 1
.\.conda\python.exe scripts/test_dynamic_scenarios.py --scenarios head_on --repeat 1 --output ("docs/dynamic_test_runs/head_on_" + (Get-Date -Format yyyyMMdd_HHmmss_fff))
```

The legacy `dynamic` test inserts a stationary box in the original factory; named advanced scenarios exercise continuously moving traffic. Output directories must be new. The benchmark discovers the active simulation container through Compose, so resource sampling works regardless of the checkout directory name. Raw evidence keeps original container names and machine paths for provenance.

See [advanced commands](dynamic_commands.md), [measurement definitions](dynamic_factory_architecture.md), [measured results](dynamic_navigation_results.md) and the [four-stage presentation guide](four_stage_demo_script.md). A full matrix rerun is not needed to inspect published results.

## Stop and restart

```powershell
docker compose logs -f simulation
docker compose stop simulation
# Clean project restart:
docker compose down
.\scripts\run_demo.ps1 -DynamicFactory -Perception
```

Only this Compose project's containers are affected. No code or evidence is removed by `down`. An unchanged launcher invocation may reuse the previous world, so use `down` when a fresh Loading start is required.

## Linux with Docker

The same mounted `/workspace` runtime can be used without Windows Conda or PowerShell. The following starts the original factory:

```bash
docker compose build
docker compose up -d
docker compose exec -T simulation bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /workspace/scripts/send_goal.py --x 4.5 --y 0 --initial-x -4.5 --initial-y 0'
docker compose down
```

For advanced traffic without perception:

```bash
docker compose build
docker run --rm --mount "type=bind,source=$PWD,target=/workspace" factorynav-ai:jazzy python3 /workspace/setup/fetch_person.py
FACTORYNAV_GUI=true FACTORYNAV_TRAFFIC=full docker compose -f compose.yaml -f compose.dynamic.yaml up -d --no-build
```

The Linux Docker commands follow the same Compose definitions; the recorded host launch checks were performed from Windows PowerShell. Native ROS installs require the equivalent Noble/Jazzy dependencies and are not an independently validated installation path.
