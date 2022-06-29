# Computational Design of Passive Grippers

Public code release for ["Computational Design of Passive Grippers"](https://homes.cs.washington.edu/~milink/passive-gripper/), presented at SIGGRAPH 2022 and authored by Milin Kodnongbua, Ian Good, Yu Lou, Jeffrey Lipton, and Adriana Schulz.

# Installation

```bash
git clone https://github.com/milmillin/passive-gripper.git
cd passive-gripper
git submodule update --init --recursive
```
## Dependencies

We use [vcpkg](https://github.com/microsoft/vcpkg) to handle dependencies. Install the following packages:

```bash
vcpkg install embree3:x64-[windows|linux]
vcpkg install qhull:x64-[windows|linux]
vcpkg install cgal:x64-[windows|linux]
vcpkg install nlopt:x64-[windows|linux]
```

## Compilation

The compilation is done using the standard cmake routine. Inside the root project directory, run:

```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake
cmake --build .

```

# Usage

There are two executables for this project `passive-gripper` and `psg-batch`.

## `passive-gripper`: User Interface

- Import mesh
- Specify intial robot pose/contact points/settings
- Save/Load configuration

*Work in progress*

## `psg-batch`: Batch Optimization

```bash
./psg-batch [-r WORKING_DIR] PSGTESTS
```

*Work in progress*
