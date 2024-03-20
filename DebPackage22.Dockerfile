# Build .deb file for Debian 11, Ubuntu 20.04, Ubuntu 22.04
FROM ubuntu:20.04 AS build-stage

WORKDIR /LimeSuiteNG/source

COPY install_dependencies.sh install_dependencies.sh

ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt-get install -y --no-install-recommends \
        dpkg-dev \
        debhelper \
    && \
    ./install_dependencies.sh -y && \
    rm -rf /var/lib/apt/lists/*

COPY amarisoft-plugin/ amarisoft-plugin/
COPY cmake/ cmake/
COPY debian/ debian/
COPY debian/control_soapy0.7 debian/control
COPY Desktop/ Desktop/
COPY external/ external/
COPY udev-rules/ udev-rules/
COPY SoapyLMS7/ SoapyLMS7/
COPY Changelog.txt Changelog.txt
COPY CMakeLists.txt CMakeLists.txt
COPY README.md README.md
COPY src/ src/

RUN dpkg-buildpackage --build=binary --no-sign -d

FROM scratch AS export-stage
COPY --from=build-stage /LimeSuiteNG/*.* /
