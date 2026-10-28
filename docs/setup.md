# Setup

## Windows project environment (verified)

Use PowerShell in `D:\cele`:

```powershell
.\setup\create_env.ps1
. .\setup\activate.ps1
python -c "import sys, numpy, yaml; print(sys.executable, sys.version); print(numpy.__version__, yaml.__version__)"
python -m pip check
```

`create_env.ps1` reuses an existing environment. It does not update Conda base or
change the user's PowerShell profile. Activation applies to the current shell.

The explicit lock file is specific to Windows x86-64:

```powershell
& 'D:\ana3\Scripts\conda.exe' create --prefix .\.conda --file .\conda-win-64.lock.txt --yes
```

Do not use that Windows lock file to create a Linux ROS environment.

## Ubuntu 24.04 robotics container (verified)

The existing WSL Ubuntu 26.04 is kept intact. Docker supplies the Ubuntu 24.04
runtime required by the selected Jazzy/Harmonic binary packages. Package names
follow the official [Nav2 quickstart](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/).

```powershell
docker desktop start
.\scripts\run_demo.ps1 -Build
```

The graphical fallback desktop is available on this machine at
[Gazebo/RViz desktop](http://localhost:6080/vnc.html?autoconnect=1&resize=scale).
The web port is bound only to `127.0.0.1`. The measured Xvfb renderer is Mesa
llvmpipe, OpenGL 4.5, with hardware acceleration reported as unavailable.
Gazebo, RViz, sensors, autonomous goals, and repeated obstacle avoidance passed
using that software fallback.

Inspect logs and stop only this project's processes:

```powershell
docker compose logs -f simulation
docker compose down
```

For a native Ubuntu 24.04 installation with ROS dependencies already installed:

```bash
bash scripts/run_demo.sh --baseline --gui
```

For the container's virtual desktop:

```bash
bash scripts/run_demo.sh --baseline --virtual-display
```

## Verified scenarios

```powershell
.\scripts\run_tests.ps1 -Scenario baseline -Repeat 1
.\scripts\run_tests.ps1 -Scenario factory -Repeat 1
.\scripts\run_tests.ps1 -Scenario dynamic -Repeat 2
```

Factory coordinates: start `(-4.5, 0.0)`, goal `(4.5, 0.0)`. The official
baseline test uses start `(-2.0, -0.5)` and goal `(0.0, 1.7)`. These commands
reset the project container before each run and validate the actual action
result, lifecycle, sensor input, and transforms.

Optional perception uses `scripts/run_demo.ps1 -Perception -Build`. It selects
`compose.perception.yaml`, which adds the RGB camera and CPU YOLO environment.
See the README for the asset fetch, person spawn, and verification commands.

The optional image passed `pip check`. The helper caches pinned official PyPI
wheels with SHA256 checks before building; verified cached files can be reused
without registry calls. `setup/perception_wheels.lock.json` records wheel sources
and hashes, and `docs/perception-python.lock.txt` records the installed packages.
The Windows Conda tooling environment remains separate from this Linux runtime.

## Advanced dynamic factory

After the existing base/perception images and person asset cache are ready:

```powershell
.\.conda\python.exe setup/fetch_person.py
.\scripts\run_demo.ps1 -DynamicFactory -Perception
```

The launcher builds the native MPPI critic against the installed Jazzy headers
when needed. Omit `-Perception` for LiDAR navigation alone. Use `-Headless` for
benchmarks and `-Traffic none` before a benchmark that creates its own actors.
See [advanced commands](dynamic_commands.md), [measurement definitions](dynamic_factory_architecture.md)
and [measured results](dynamic_navigation_results.md). The original launchers
above continue to select the original factory.

For a Linux machine with Docker, the core workflow is also available without
PowerShell:

```bash
docker compose build
docker compose up -d
docker compose exec simulation bash -lc 'source /opt/ros/jazzy/setup.bash; python3 scripts/send_goal.py --x 4.5 --y 0 --initial-x -4.5 --initial-y 0 --obstacle-after-m 1.0'
docker compose down
```
