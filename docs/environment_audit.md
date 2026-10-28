# Environment audit

Observed on 2026-10-28 (Asia/Kolkata).

| Component | Observed result |
| --- | --- |
| Workspace | `D:\cele`, initially empty |
| Host | Windows 11 Home Single Language, build 26200 |
| Host RAM | About 15.7 GiB total; 4.4 GiB free at audit time |
| Conda | `D:\ana3\Scripts\conda.exe`, version 25.5.1 |
| Conda base Python | 3.13.5; base is left unchanged |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU, 8188 MiB, driver 560.94 |
| WSL | Ubuntu and docker-desktop registered as WSL2 distributions |
| Ubuntu | **26.04.1 LTS**, Python 3.14.4 |
| WSL kernel | 6.18.54.1-microsoft-standard-WSL2 |
| WSL GPU | `nvidia-smi` succeeds and reports the RTX 4060 |
| CUDA computation | Not tested; GPU visibility alone does not prove PyTorch CUDA works |
| WSLg | `/mnt/wslg` exists; DISPLAY=:0 and WAYLAND_DISPLAY=wayland-0 |
| Graphics rendering | Gazebo and RViz verified in Xvfb/noVNC; Mesa llvmpipe, OpenGL 4.5, not accelerated |
| WSL memory | 7.6 GiB RAM and 2 GiB swap |
| Free disk | C: about 138.8 GiB; D: about 19.5 GiB |
| ROS and Gazebo | Absent in host Ubuntu; installed and verified in the Ubuntu 24.04 container |
| Linux administration | User `abhi_08`; passwordless sudo unavailable |
| Docker | CLI installed; Docker Desktop engine initially stopped |
| Local environment | `D:\cele\.conda`: Python 3.12.16, NumPy 1.26.4, PyYAML 6.0.3 |
| Environment verification | Activation, interpreter prefix, NumPy operation, YAML parsing, and `pip check` passed |
| Container runtime | Docker 29.8.2 started; official ROS image reports Ubuntu 24.04.5 and Python 3.12.3 |
| Container GPU | NVIDIA runtime exposes RTX 4060 and `/dev/dxg` |

## Compatibility decision

ROS 2 Jazzy's Ubuntu binary target is Ubuntu 24.04 Noble. Gazebo Harmonic
binaries support Noble and Jammy. The current Ubuntu 26.04 distribution cannot
use the normal Jazzy/Noble apt setup safely. Do not add Noble package repositories
to this distribution or replace its Python.

The requested Windows Conda environment belongs at `D:\cele\.conda` and uses
Python 3.12 for local tooling. It is a Windows environment, so it cannot load
Linux `rclpy` extensions or run the Linux ROS stack. A separate Ubuntu 24.04
runtime is needed for the requested Jazzy/Harmonic simulation. A container or
side-by-side WSL distribution can provide that without replacing Ubuntu 26.04.

The selected runtime is a project Docker container based on the official
`ros:jazzy-ros-base-noble` image. A read-only mount of Ubuntu's WSLg directory
failed with `distro-services/ubuntu.sock: no such file or directory`, indicating
that Docker's WSL integration for this distribution is unavailable. The project
therefore provides a local-only noVNC/Xvfb desktop fallback. Gazebo/RViz rendering
and navigation were verified using Mesa llvmpipe. NVIDIA compute visibility does
not prove hardware-accelerated OpenGL in that virtual desktop.

No operating system upgrade, downgrade, or replacement has been performed.

## Official references

- [ROS target platforms (REP-2000)](https://reps.openrobotics.org/rep-2000/)
- [Gazebo Harmonic Ubuntu installation](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [Nav2 Jazzy quickstart](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/)
- [Conda environments and prefixes](https://docs.conda.io/projects/conda/en/25.5.x/user-guide/tasks/manage-environments.html)
- [Ultralytics YOLO26](https://docs.ultralytics.com/models/yolo26/)
