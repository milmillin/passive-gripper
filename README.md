# Passive Gripper

This is a passive gripper project.

# Installation

```bash
git clone https://github.com/milmillin/passive-gripper-v2.git
cd passive-gripper-v2
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

There are two target for this project `passive-gripper` and `psg-batch`.

## `passive-gripper`: User Interface

- Import mesh
- Specify intial robot pose/contact points/settings
- Save/Load configuration

## `psg-batch`: Batch Optimization

```bash
./psg-batch [-r WORKING_DIR] PSGTESTS
```

### `.psgtests` file

Description of test objects.

```
%PSGTESTS 1
N_OBJ
NAME INPUT_PSG CP_FMT N_FILES OUT_FMT
NAME INPUT_PSG CP_FMT N_FILES OUT_FMT
NAME INPUT_PSG CP_FMT N_FILES OUT_FMT
```

The file starts with `%PSGTEST 1` signature. The next line contains the number of objects (`N_OBJ`).
Each of the next following `N_OBJ` lines describes each object.

- `NAME`: name of test case
- `INPUT_PSG`: name of psg file
- `CP_FMT`: `printf`-styled filename of cp files
- `N_FILES`: number of cp files for the objects
- `OUT_FMT`: `printf`-styled filename of output files

**Example**

```
%PSGTESTS 1
1
topkey topkey3_input.psg topkey_%03d.cp 100 topkey_optd_%03d
```
