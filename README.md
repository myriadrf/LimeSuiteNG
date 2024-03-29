# Lime Suite NG

**Build Status**: ![Cross platform build status](https://github.com/myriadrf/LimeSuiteNG/actions/workflows/cmake.yml/badge.svg)

Lime Suite NG is a collection of software supporting several hardware platforms
based on the LMS7002M transceiver RFIC, such as [LimeSDR](https://wiki.myriadrf.org/LimeSDR) family. It contains the
following components:
* **limesuiteng** library that provides C++ API ;
* **limeGUI** graphical user interface for manipulating low-level chip functions and board settings, displaying FFT;

Plugins for external software:
* **SoapyLMS** LimeSDR devices support for SoapySDR;

## Supported devices
* [LimeSDR-USB](https://wiki.myriadrf.org/LimeSDR-USB)
* [LimeSDR Mini](https://wiki.myriadrf.org/LimeSDR-Mini)
* [LimeSDR Mini v2](https://limesdr-mini.myriadrf.org/index.html)
* [LimeSDR XTRX](https://limesdr-xtrx.myriadrf.org/)

## Installing

### Linux:
* Building from source:
```
git clone https://github.com/myriadrf/LimeSuiteNG
cd LimeSuiteNG
sudo ./install_dependencies.sh
cmake -B build && cd build
make
sudo make install
```

* Installing from packages:
**TODO**

### Windows:
**TODO**

## Documentation
Information about LimeSDR boards:
* https://wiki.myriadrf.org/LimeSDR

## Help and support
The discourse forum is a good way to find help and discuss topics:
* https://discourse.myriadrf.org/
