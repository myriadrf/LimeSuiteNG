# Build .deb file for Debian 11, Ubuntu 20.04, Ubuntu 22.04
FROM ubuntu:20.04 AS build-stage

WORKDIR /LimeSuiteNG/source

COPY install_dependencies.sh install_dependencies.sh

ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt-get install -y --no-install-recommends \
        debhelper \
        dh-python \
        dpkg-dev \
    && \
    ./install_dependencies.sh -y && \
    rm -rf /var/lib/apt/lists/*

COPY cmake/ cmake/
COPY docs/doxygen docs/doxygen
COPY debian/ debian/
COPY external/ external/
COPY udev-rules/ udev-rules/
COPY plugins/ plugins/
COPY Changelog.txt Changelog.txt
COPY CMakeLists.txt CMakeLists.txt
COPY README.md README.md
COPY GUI/ GUI/
COPY embedded/ embedded/
COPY src/ src/

RUN patch debian/control < debian/control.soapy0.7.patch
RUN patch debian/control < debian/control.no_kernel_driver.patch
RUN patch debian/rules < debian/rules.docker.patch
RUN dpkg-buildpackage --build=binary --no-sign -d

FROM scratch AS export-stage
COPY --from=build-stage /LimeSuiteNG/*.* /
