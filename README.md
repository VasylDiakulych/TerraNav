## Project summary
TerraNav - project made in C++ about navigation in partially observable environment.

### Setting
You're given a "satellite image" of the unknown planet, where a drone is landed somewhere. It must navigate through regions to reach the goal. It can observe everything on satellite level, but on a local level it can only use its sensors "on the ground", so the observability is partial.

### Solution
Project implements 2-level hierarchical navigation, where normal A* is used to navigate on satellite level between "gates" connecting regions and D*-Lite, MPAA* or Naive A* are used to navigate in condition of partial observability inside regions, on "local" level.

### Features
Project features several side hustles I decided to implement in addition to the main part. Several of the features are:
1. Procedural terrain generator
2. Benchmarking for navigation algorithms

### Dependencies
Use the following commands, depending on your Linux distribution, to install them.
#### Ubuntu / Debian
```bash
sudo apt install git cmake build-essential \
  libgl1-mesa-dev libx11-dev libxcursor-dev \
  libxi-dev libxinerama-dev libxrandr-dev
```

#### Fedora
```bash
sudo dnf install git cmake gcc-c++ make \
  libX11-devel libXcursor-devel libXrandr-devel \
  libXi-devel libXinerama-devel mesa-libGL-devel
```

#### Arch
```bash
sudo pacman -S git cmake gcc make \
  libx11 libxcursor libxrandr libxi libxinerama mesa
```
### How to run
There's a `run.sh` script in root directory. After downloading all dependencies above, clone the project repository and run it from terminal via "`./run.sh`". 

## IMPORTANT NOTE:
Project was built for Linux and is not tested for other operating systems. If you use Windows, please install any Linux distribution, if you expect for project to work correctly.