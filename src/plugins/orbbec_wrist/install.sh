#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
SOURCE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd -P)"
SDK_RELEASE="v2.10.6-EGO-20261019"
SDK_NAME="OrbbecSDK_v2.10.6_202610191449_d49af169_linux_x86_64"
SDK_ARCHIVE="$SDK_NAME.tar.gz"
SDK_URL="https://github.com/orbbec/OrbbecSDK_v2-EGO/releases/download/$SDK_RELEASE/$SDK_ARCHIVE"
SDK_SHA256="378761b4331b7c6ce75d3c7dc61817ed27a5cacb3265a77477587c4d9e3bf11a"
SDK_CACHE="$SOURCE_ROOT/build/wrist-sdk/$SDK_RELEASE"

die() {
    echo "install.sh: $*" >&2
    exit 1
}

status() {
    printf '\n============================================================\n'
    printf '[%s] %s\n' "$1" "$2"
    printf '============================================================\n'
}

usage() {
    cat <<'EOF'
Usage: ./src/plugins/orbbec_wrist/install.sh [options]

Install native build dependencies, fetch the pinned Linux x86_64 Orbbec SDK,
install host udev rules, and build the Orbbec Wrist plugin as the invoking
user. Requires uv and CMake 3.24 or newer. Run from a source checkout, without
sudo; only missing system packages and host udev setup request sudo.

Options:
  Value options accept both --option value and --option=value.
  --sdk-root PATH       Use an extracted SDK instead of downloading it.
                        ORBBEC_SDK_ROOT also selects a local SDK.
  --container           Skip host udev setup (also detected automatically).
  --skip-deps           Use existing system dependencies (uv/CMake still required).
  --skip-udev           Keep existing host udev configuration.
  --preset NAME         py3.11 (default), py3.12 or py3.13.
  --jobs N              Parallel build jobs.
  --preview             Build live Color preview and install its SDL2/FFmpeg dependencies.
  --install-prefix PATH Also install component orbbec_wrist at PATH (without SDK files).
  -h, --help            Show this help.

Downloaded SDKs are SHA-256 checked and unpacked under build/wrist-sdk/.
Local SDKs must supply the Orbbec Wrist APIs and the 2bc5:1203 USB udev rule.
SDK checks do not certify Orbbec Wrist streaming; build and hardware validation remain required.
The vendor udev file covers multiple devices, including Orbbec Wrist USB 2bc5:1203.
EOF
}

sdk_root="${ORBBEC_SDK_ROOT:-}"
preset=py3.11
jobs=""
install_prefix=""
container=0
skip_deps=0
skip_udev=0
preview=0
while (( $# )); do
    case "$1" in
        --sdk-root=*|--preset=*|--jobs=*|--install-prefix=*)
            set -- "${1%%=*}" "${1#*=}" "${@:2}"
            ;;
    esac
    case "$1" in
        --sdk-root|--preset|--jobs|--install-prefix)
            [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 requires a value."
            case "$1" in
                --sdk-root) sdk_root="$2" ;;
                --preset) preset="$2" ;;
                --jobs) jobs="$2" ;;
                --install-prefix) install_prefix="$2" ;;
            esac
            shift 2
            ;;
        --container) container=1; shift ;;
        --skip-deps) skip_deps=1; shift ;;
        --skip-udev) skip_udev=1; shift ;;
        --preview) preview=1; shift ;;
        --help|-h) usage; exit 0 ;;
        *) die "Unknown option '$1'; run with --help." ;;
    esac
done

[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || die "This installer supports Linux x86_64 only."
[[ -f "$SOURCE_ROOT/src/plugins/orbbec_wrist/CMakeLists.txt" ]] || die "Installation requires an Isaac Capture source checkout."
case "$preset" in
    py3.11|py3.12|py3.13) ;;
    *) die "Unknown preset '$preset'." ;;
esac
[[ -z "$jobs" || "$jobs" =~ ^[1-9][0-9]*$ ]] || die "--jobs must be a positive integer."
if [[ -f /.dockerenv ]] || grep -qE '(docker|containerd|kubepods)' /proc/1/cgroup 2>/dev/null; then
    container=1
fi
run_as_root() {
    if (( EUID == 0 )); then
        "$@"
    else
        command -v sudo >/dev/null 2>&1 || die "sudo is required for system setup; use --skip-deps and --skip-udev on a prepared host."
        sudo "$@"
    fi
}

if (( !skip_deps )); then
    command -v apt-get >/dev/null 2>&1 && command -v dpkg-query >/dev/null 2>&1 \
        || die "Automatic dependencies require apt-get; install dependencies manually and use --skip-deps."
    packages=(
        build-essential ca-certificates cmake ccache clang-format-14 curl git
        patchelf pkg-config libudev-dev libusb-1.0-0-dev nlohmann-json3-dev
        libssl-dev ffmpeg jq usbutils glslang-tools libegl-dev libvulkan-dev
        libwayland-dev libx11-dev libxcursor-dev libxext-dev libxi-dev
        libxinerama-dev libxkbcommon-dev libxrandr-dev wayland-protocols
    )
    missing=()
    if (( preview )); then
        packages+=(libsdl2-dev libavcodec-dev libavutil-dev libswscale-dev)
    fi
    for package in "${packages[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -Fq 'install ok installed'; then
            missing+=("$package")
        fi
    done
    if (( ${#missing[@]} )); then
        echo "Installing missing system packages: ${missing[*]}"
        run_as_root apt-get update
        run_as_root apt-get install -y "${missing[@]}"
    else
        echo "==> System dependencies are already installed."
    fi
    status OK "Step 1/5: System dependencies are ready."
else
    status SKIP "Step 1/5: System dependency installation skipped (--skip-deps)."
fi

command -v uv >/dev/null 2>&1 || die "Install uv, a prerequisite for the Isaac Capture source build, then rerun this command."
command -v cmake >/dev/null 2>&1 || die "Install CMake 3.24 or newer."
cmake_version="$(cmake --version | awk 'NR == 1 {print $3}')"
[[ "$(printf '%s\n%s\n' 3.24 "$cmake_version" | sort -V | head -n 1)" == 3.24 ]] \
    || die "CMake $cmake_version is too old; install CMake 3.24 or newer."
status OK "Step 2/5: Build prerequisites checked (uv found, CMake $cmake_version)."

validate_sdk() {
    local root="$1" path
    for path in include/libobsensor/ObSensor.hpp lib/OrbbecSDKConfig.cmake lib/libOrbbecSDK.so.2 shared/99-obsensor-libusb.rules; do
        [[ -f "$root/$path" ]] || die "Invalid SDK root '$root': missing $path. Use the $SDK_RELEASE package or a compatible local SDK."
    done
    [[ -d "$root/lib/extensions" ]] || die "Invalid SDK root '$root': missing lib/extensions."
    awk '
        /^[[:space:]]*#/ {next}
        /ATTRS[{]idVendor[}][[:space:]]*==[[:space:]]*"2bc5"/ &&
        /ATTRS[{]idProduct[}][[:space:]]*==[[:space:]]*"1203"/ {found=1}
        END {exit !found}
    ' "$root/shared/99-obsensor-libusb.rules" \
        || die "SDK udev rules lack the Orbbec Wrist USB device 2bc5:1203. Select a package with Orbbec Wrist support."
}

work=""
trap '[[ -z "$work" ]] || rm -rf "$work"' EXIT
if [[ -n "$sdk_root" ]]; then
    validate_sdk "$sdk_root"
    sdk_root="$(cd "$sdk_root" && pwd -P)"
    echo "==> Using local SDK: $sdk_root"
    status OK "Step 3/5: Local SDK validation succeeded (download skipped)."
    printf 'SDK directory: %s\n' "$sdk_root"
else
    command -v curl >/dev/null 2>&1 || die "curl is required to download the Orbbec SDK."
    command -v sha256sum >/dev/null 2>&1 || die "sha256sum is required to verify the Orbbec SDK."
    mkdir -p "$SDK_CACHE"
    archive="$SDK_CACHE/$SDK_ARCHIVE"
    work="$(mktemp -d "$SDK_CACHE/.install.XXXXXX")"
    if [[ -f "$archive" ]]; then
        cached_sha256="$(sha256sum "$archive" | awk '{print $1}')"
        if [[ "$cached_sha256" != "$SDK_SHA256" ]]; then
            echo "==> Cached SDK SHA-256 mismatch; downloading a verified replacement."
            rm -f "$archive"
        fi
    fi
    if [[ ! -f "$archive" ]]; then
        echo "==> Downloading Orbbec SDK $SDK_RELEASE"
        curl --fail --location --show-error --retry 3 --connect-timeout 30 --max-time 600 \
            "$SDK_URL" --output "$work/$SDK_ARCHIVE"
        actual_sha256="$(sha256sum "$work/$SDK_ARCHIVE" | awk '{print $1}')"
        [[ "$actual_sha256" == "$SDK_SHA256" ]] \
            || die "SDK SHA-256 mismatch: expected $SDK_SHA256; got $actual_sha256."
        mv "$work/$SDK_ARCHIVE" "$archive"
        status OK "SDK download and SHA-256 verification succeeded."
    else
        echo "==> Reusing SHA-256 verified SDK archive: $archive"
        status OK "Cached SDK archive SHA-256 verification succeeded (download skipped)."
    fi
    printf 'SDK archive: %s\n' "$archive"
    # Recreate the SDK from the verified archive; extracted files may have changed.
    tar -xzf "$archive" -C "$work"
    validate_sdk "$work/$SDK_NAME"
    sdk_root="$SDK_CACHE/$SDK_NAME"
    rm -rf "$sdk_root"
    mv "$work/$SDK_NAME" "$sdk_root"
    status OK "Step 3/5: SDK extraction and validation succeeded; SDK setup complete."
    printf 'SDK directory: %s\n' "$sdk_root"
fi

if (( container )); then
    echo "==> Container: skipping host udev setup."
    status SKIP "Step 4/5: Host udev setup skipped (container)."
    echo "Install $sdk_root/shared/99-obsensor-libusb.rules on the host for Orbbec Wrist USB 2bc5:1203, then reconnect the camera."
elif (( skip_udev )); then
    status SKIP "Step 4/5: Host udev setup skipped (--skip-udev)."
    echo "Keeping existing udev rules; Orbbec Wrist USB 2bc5:1203 must already be accessible."
else
    command -v udevadm >/dev/null 2>&1 || die "Install udev or use --skip-udev on a prepared host."
    echo "Installing the vendor USB udev file (includes rules for other Orbbec devices)."
    run_as_root install -m 0644 "$sdk_root/shared/99-obsensor-libusb.rules" /etc/udev/rules.d/99-obsensor-libusb.rules
    run_as_root udevadm control --reload-rules
    run_as_root udevadm trigger --subsystem-match=usb --attr-match=idVendor=2bc5 --attr-match=idProduct=1203
    status OK "Step 4/5: USB udev rules installed and reloaded."
    printf 'udev rules: /etc/udev/rules.d/99-obsensor-libusb.rules\n'
    echo "Unplug and reconnect Orbbec Wrist to apply the permissions."
fi

build_args=(build --sdk-root "$sdk_root" --preset "$preset")
(( !preview )) || build_args+=(--preview)
[[ -z "$jobs" ]] || build_args+=(--jobs "$jobs")
[[ -z "$install_prefix" ]] || build_args+=(--install-prefix "$install_prefix")
echo "==> Building the Orbbec Wrist plugin."
"$SCRIPT_DIR/orbbec_wrist.sh" "${build_args[@]}"
status OK "Step 5/5: Orbbec Wrist plugin build succeeded."
printf 'Plugin: %s/build-wrist-%s/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin\n' "$SOURCE_ROOT" "$preset"
if [[ -n "$install_prefix" ]]; then
    status OK "Plugin component installation succeeded."
    printf 'Install prefix: %s\n' "$install_prefix"
fi
status OK "Orbbec Wrist setup completed successfully."
