* Setup

```
git clone https://github.com/myriadrf/LimeSDR-Micro_FW
cd LimeSDR-Micro_FW
git submodule update --init --recursive
```

Project compilation requires Arm toolchain, it will be downloaded during cmake configuration step

* Build steps:
```
mkdir build
cd build
cmake ..
make
```
