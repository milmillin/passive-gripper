# Passive Gripper

## Installation
```bash
git clone https://github.com/milmillin/passive-gripper.git
git submodule update --init --recursive
```
## Dependencies

- [Embree](https://www.embree.org/downloads.html) needs manual installation

#### Windows

```cmd
vcpkg install embree3:x64-windows
```

#### Linux

1. Download and extract [embree-3.12.1.x86_64.linux.tar.gz](https://github.com/embree/embree/releases/download/v3.12.1/embree-3.12.1.x86_64.linux.tar.gz) (other versions may be fine, but they are not tested)
2. Edit the line 9 of `embree-vars.sh`. There should be a space after the `while` keyword, but it's missing in the original source code. (Hint: in `while ([ -h "${SCRIPT_PATH}" ])`)
3. Run `embree-vars.sh`
4. Set `embree_DIR` to the directory where Embree is located (Hint: `export embree_DIR=<your directory>`)

## Compile

Compile this project using the standard cmake routine:

```bash
mkdir build
cd build
cmake ..
cmake --build .

```

The following warning is fine if OpenMP is not supported by your compiler:

    warning: ignoring ‘#pragma omp parallel’ [-Wunknown-pragmas]