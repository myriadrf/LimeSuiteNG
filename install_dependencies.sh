#!/bin/bash

set -eu -o pipefail # fail on error and report it, debug all lines

if [ "$EUID" -ne 0 ]; then
  echo "You should run this script as root"
  exit 1
fi

ID=""
ID_LIKE=""
TYPE=""
. /etc/os-release

if [[ $ID == "ubuntu" || $ID == "debian" ]]; then
  TYPE=$ID
fi

if [[ $TYPE == "" ]]; then
  LIKE=${ID_LIKE%% *}

  if [[ $LIKE == "ubuntu" ]]; then
    TYPE="ubuntu"
  elif [[ $LIKE == "debian" ]]; then
    TYPE="debian"
  fi
fi

if [[ $TYPE == "" ]]; then
  echo "Current distribution is very likely currently unsupported, aborting."
  exit 1
fi

libwx="libwxgtk3.2-dev"

if [[ ($TYPE == "ubuntu" && ${VERSION_ID:0:2} -le 22) || ($TYPE == "debian" && $VERSION_ID -le 11) ]]; then
  libwx="libwxbase3.0-dev libwxgtk3.0-gtk3-dev"
fi

headers=""

if [[ $TYPE == "ubuntu" ]]; then
  if [[ $VERSION_ID == "23.10" ]]; then
    VERSION_ID=22.04
  fi
  headers="linux-headers-generic-hwe-${VERSION_ID}"
elif [[ $TYPE == "debian" ]]; then
  headers="linux-headers-generic"
fi

yes=""
if [[ ${1-} == "-y" ]]; then
  yes="-y"
fi

apt-get install $yes --no-install-recommends \
  build-essential \
  cmake \
  $headers \
  libsoapysdr-dev \
  libusb-1.0-0-dev \
  $libwx
