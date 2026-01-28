#!/usr/bin/env bash

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

EARTHLY_VERSION=v0.8.17-rc-2

case $(uname -s) in
    Linux*)     KERNEL=linux;;
    Darwin*)    KERNEL=darwin;;
    CYGWIN*)    KERNEL=windows;;
    MINGW*)     KERNEL=windows;;
    MSYS_NT*)   KERNEL=windows;;
    *)          echo "Unknown platform"; exit 2;;
esac

case $(uname -m) in
    x86_64)           ARCH=amd64;;
    arm64|aarch64)    ARCH=arm64;;
    *)                echo "Unknown architecture"; exit 2;;
esac

pushd ${SCRIPT_DIR}

[ -d ./earthly-src ] && rm -rf ./earthly-src
git clone -b ${EARTHLY_VERSION} https://github.com/EarthBuild/earthbuild.git earthly-src
git -C ./earthly-src apply --3way ${SCRIPT_DIR}/earthly.patch

[ -f ./earthly-bootstrap ] && rm ./earthly-bootstrap
wget -O ./earthly-bootstrap https://github.com/EarthBuild/earthbuild/releases/download/${EARTHLY_VERSION}/earthly-${KERNEL}-${ARCH}
chmod +x ./earthly-bootstrap

./earthly-bootstrap ./earthly-src+for-own
ln -sTf ./earthly-src/build/own/earthly ./earthly

popd
