#!/bin/bash
# Installs the Huateng Vision Linux SDK (MindVision MVSDK) for huateng_camera_ros.
#
#   sudo scripts/install_sdk.sh                          # download from huatengvision.com and install
#   sudo scripts/install_sdk.sh linuxSDK_V2.1.0.49.tar.gz # install from a downloaded archive
#
# Installs to $HUATENG_SDK_DIR (default /opt/huatengvision_SDK):
#   include/  CameraApi.h, CameraDefine.h, CameraStatus.h
#   lib/      libMVSDK.so for this machine (x86_64, aarch64, armv7, i686), registered with ldconfig
# plus the udev rules of the SDK (needed for USB cameras only).
#
# The SDK is not part of this repository; it is downloaded from the manufacturer (no login needed).
set -euo pipefail

SDK_URL="https://www.huatengvision.com/attachment/down/13/linuxSDK_V2.1.0.49(202511141513).tar.gz"
SDK_SHA256="9624d193a908d94a9850400346cf1ab8095437a040e381b2fd39029b0183dd8a"
PREFIX="${HUATENG_SDK_DIR:-/opt/huatengvision_SDK}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Please run as root: sudo $0 $*" >&2
  exit 1
fi

case "$(uname -m)" in
  x86_64)          ARCH=x64 ;;
  aarch64|arm64)   ARCH=arm64 ;;
  armv7l|armv7*)   ARCH=arm ;;      # hard float (Raspberry Pi OS 32 bit); use arm_softfp manually if needed
  i?86)            ARCH=x86 ;;
  *) echo "Unsupported architecture: $(uname -m)" >&2; exit 1 ;;
esac

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

if [ $# -ge 1 ]; then
  ARCHIVE="$1"
  [ -f "$ARCHIVE" ] || { echo "Archive not found: $ARCHIVE" >&2; exit 1; }
else
  ARCHIVE="$TMP/sdk.tar.gz"
  echo "Downloading the SDK (58 MB) from huatengvision.com ..."
  if command -v curl >/dev/null; then
    curl -fL --retry 3 -A "Mozilla/5.0" -o "$ARCHIVE" "$SDK_URL"
  else
    wget -O "$ARCHIVE" --user-agent="Mozilla/5.0" "$SDK_URL"
  fi
  echo "$SDK_SHA256  $ARCHIVE" | sha256sum -c - >/dev/null || {
    echo "Checksum of the download does not match V2.1.0.49. The manufacturer may have published a new version:" >&2
    echo "download it from https://www.huatengvision.com/download/1/ and run: sudo $0 <archive>" >&2
    exit 1
  }
fi

tar xzf "$ARCHIVE" -C "$TMP" --wildcards '*/include/*' '*/lib/*' '*/88-mvusb.rules' '*/99-mvusb.rules' \
  '*/LinuxSDK Document.pdf' 2>/dev/null || tar xzf "$ARCHIVE" -C "$TMP"
SRC="$(dirname "$(find "$TMP" -name CameraApi.h -path '*/include/*' | head -1)")/.."
[ -f "$SRC/lib/$ARCH/libMVSDK.so" ] || { echo "lib/$ARCH/libMVSDK.so not found in the archive" >&2; exit 1; }

mkdir -p "$PREFIX/include" "$PREFIX/lib" "$PREFIX/doc"
cp "$SRC"/include/*.h "$PREFIX/include/"
install -m 755 "$SRC/lib/$ARCH/libMVSDK.so" "$PREFIX/lib/libMVSDK.so"
ln -sf libMVSDK.so "$PREFIX/lib/libmvsdk.so"   # some projects link with -lmvsdk
[ -f "$SRC/LinuxSDK Document.pdf" ] && cp "$SRC/LinuxSDK Document.pdf" "$PREFIX/doc/"
echo "$PREFIX/lib" > /etc/ld.so.conf.d/huatengvision.conf
ldconfig

if [ -d /etc/udev/rules.d ]; then
  cp "$SRC/88-mvusb.rules" "$SRC/99-mvusb.rules" /etc/udev/rules.d/ 2>/dev/null || true
  command -v udevadm >/dev/null && udevadm control --reload-rules 2>/dev/null || true
fi

VERSION="$(basename "$(dirname "$(find "$TMP" -maxdepth 2 -name install.sh | head -1)")" 2>/dev/null || true)"
echo "Installed ${VERSION:-Huateng SDK} ($ARCH) to $PREFIX"
echo "Build the driver as described in README.md (catkin or colcon)."
