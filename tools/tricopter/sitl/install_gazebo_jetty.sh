#!/usr/bin/env bash
# Native Ubuntu install from Gazebo's official signed repository.
set -euo pipefail
if (( EUID != 0 )); then
    echo "Run: sudo bash tools/tricopter/sitl/install_gazebo_jetty.sh" >&2
    exit 1
fi
. /etc/os-release
if [[ "$ID" != ubuntu || ! "$VERSION_CODENAME" =~ ^(noble|resolute)$ ]]; then
    echo "This installer supports Ubuntu 24.04/noble and 26.04/resolute." >&2
    exit 1
fi
arch=$(dpkg --print-architecture)
if [[ "$arch" != amd64 ]]; then
    echo "This setup has been prepared for amd64; got $arch." >&2
    exit 1
fi
apt-get update
apt-get install -y ca-certificates curl gnupg
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT
curl --fail --show-error --location --proto '=https' --tlsv1.2 \
    https://packages.osrfoundation.org/gazebo.gpg --output "$scratch/gazebo.gpg"
gpg --batch --show-keys "$scratch/gazebo.gpg"
key=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
source=/etc/apt/sources.list.d/gazebo-stable.list
stamp=$(date -u +%Y%m%dT%H%M%SZ)
printf 'deb [arch=%s signed-by=%s] https://packages.osrfoundation.org/gazebo/ubuntu-stable %s main\n' \
    "$arch" "$key" "$VERSION_CODENAME" > "$scratch/gazebo-stable.list"
for item in "$key" "$source"; do
    if [[ -e "$item" ]]; then cp -a -- "$item" "$item.backup-$stamp"; fi
done
install -m 0644 "$scratch/gazebo.gpg" "$key"
install -m 0644 "$scratch/gazebo-stable.list" "$source"
apt-get update
apt-get install -y gz-jetty python3-evdev build-essential cmake ninja-build pkg-config
echo "Gazebo Jetty and plugin build dependencies installed."
gz sim --versions
