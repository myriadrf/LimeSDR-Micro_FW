* Setup

```
git clone https://github.com/myriadrf/LimeSDR-Micro_FW
cd LimeSDR-Micro_FW
```

Project compilation requires Arm toolchain, it will be downloaded during cmake configuration step

Initial ADC/DAC sampling rates are determined by LA9310_SYS_CLK_FREQ

* Build steps:
```
mkdir build
cd build
cmake .. --toolchain ../armcc.cmake
make
```

* To enable external reference clock use, set it's frequency. (0 to disable external clock use)
```
cmake .. -DEXT_REF_CLK=10000000
make
```