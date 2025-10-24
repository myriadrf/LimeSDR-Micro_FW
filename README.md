* Setup

```
git clone https://github.com/myriadrf/LimeSDR-Micro_FW
cd LimeSDR-Micro_FW
git submodule update --init --recursive
```

Project compilation requires Arm toolchain, it will be downloaded during cmake configuration step

ADC/DAC sampling rates are determined by LA9310_REF_CLK_FREQ

* Build steps:
```
mkdir build
cd build
cmake .. -DLA9310_REF_CLK_FREQ=30720000
make
```
