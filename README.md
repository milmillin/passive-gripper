# Passive Gripper

## Installation
```bash
git clone https://github.com/milmillin/passive-gripper.git
git submodule update --init --recursive
```
### Dependencies
- [Embree](https://www.embree.org/downloads.html)

#### Windows
```cmd
vcpkg install embree3:x64-windows
```

#### Linux
TBA

### Compile

Compile this project using the standard cmake routine:

```bash
mkdir build
cd build
cmake ..
cmake --build .
