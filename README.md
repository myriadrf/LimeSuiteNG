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
sudo ldconfig
```

* Installing from packages:

#### apt:
Install the GPG key:
```
wget -qO - https://repo.myriadrf.org/lime-microsystems-public.gpg | gpg --dearmor | sudo tee /etc/apt/keyrings/lime-microsystems-public.gpg > /dev/null
```

Add source for your distribution:
```
echo "deb [signed-by=/etc/apt/keyrings/lime-microsystems-public.gpg] https://repo.myriadrf.org/apt stable main" | sudo tee /etc/apt/sources.list.d/repo.myriadrf.org.list
```

Update apt sources and install limesuiteng:
```
sudo apt-get update
sudo apt-get install limesuiteng
```

### Windows:
```
git clone https://github.com/myriadrf/LimeSuiteNG
cd LimeSuiteNG
cmake -B build
cmake --build build --config Release

Optionally can be installed into system, installation requires to be ran with Administrative privileges
cmake --install build --config Release
```

## Documentation
Information about LimeSDR boards:
* https://wiki.myriadrf.org/LimeSDR

## Help and support
The discourse forum is a good way to find help and discuss topics:
* https://discourse.myriadrf.org/
