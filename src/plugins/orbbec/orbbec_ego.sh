#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# One entry point for building, inspecting, recording, and verifying the
# capability-compatible Orbbec Ego camera plugin.  It intentionally never
# installs packages, SDKs, or udev rules: those operations need user approval.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
DEFAULT_PRESET="py3.11"

die() {
    echo "orbbec_ego.sh: $*" >&2
    exit 1
}

warn() {
    echo "orbbec_ego.sh: warning: $*" >&2
}

usage() {
    cat <<'EOF'
Usage:
  src/plugins/orbbec/orbbec_ego.sh doctor [--sdk-root PATH] [--preset py3.11]
  src/plugins/orbbec/orbbec_ego.sh build --sdk-root PATH [--preview] [--preset py3.11] [--jobs N] [--clean] [--install-prefix PATH]
  src/plugins/orbbec/orbbec_ego.sh capabilities [--preset py3.11] [--plugin PATH] [--device-uid UID]
  src/plugins/orbbec/orbbec_ego.sh record [--config FILE.toml] [options] [-- PLUGIN_OPTIONS...]
  src/plugins/orbbec/orbbec_ego.sh verify RUN_DIRECTORY [--preset py3.11] [--plugin PATH]
  src/plugins/orbbec/orbbec_ego.sh export-media RUN_DIRECTORY [--output DIRECTORY] [--preset py3.11]

Common record options:
  --config FILE.toml              Load reusable record settings; later CLI options override them.
  --duration SECONDS              Stop cleanly after SECONDS; omit for Ctrl-C.
  --output DIRECTORY              Default: recordings/orbbec_ego_<timestamp>.
  --format mjpg|h264|h265         Default: h264.
  --width N --height N --fps N    Default: 1600 1300 30.
  --device-uid UID                Select one enumerated device.
  --reconnect-timeout SECONDS     Wait for the same physical device after disconnect (default: 30; 0 disables).
  --reconnect-interval-ms MS      Device re-enumeration interval (default: 1000).
  --no-imu --no-audio             Do not request optional sensors.
  --preview                       Enable the plugin's SDL preview (requires a --preview build).
  --mcap-media MODE               metadata-only (default) or embedded.
  --keep-media-sidecars           Retain H.264/H.265/MJPEG and WAV beside embedded MCAP.
  --preset NAME --plugin PATH     Select a build tree or override the executable.

Examples:
  ./src/plugins/orbbec/orbbec_ego.sh doctor --sdk-root /opt/OrbbecSDK
  ./src/plugins/orbbec/orbbec_ego.sh build --sdk-root /opt/OrbbecSDK --jobs 8
  ./src/plugins/orbbec/orbbec_ego.sh build --sdk-root /opt/OrbbecSDK --preview --jobs 8
  ./src/plugins/orbbec/orbbec_ego.sh capabilities
  ./src/plugins/orbbec/orbbec_ego.sh record --duration 30
  # Capabilities also reports advertised profiles that remain uncertified.
  ./src/plugins/orbbec/orbbec_ego.sh verify recordings/orbbec_ego_20260810_120000
  ./src/plugins/orbbec/orbbec_ego.sh export-media recordings/embedded_demo

All options after -- are passed unchanged to camera_plugin_orbbec.  For example:
  ... record --duration 30 -- --bitrate=8 --dynamic-bitrate=on

The plugin validates the exact requested stream combination against the SDK;
unsupported combinations fail instead of falling back to another profile.
Encoded profiles above 30 FPS are currently rejected after SDK resolution
until sustained recording and strict-decode certification passes.
EOF
}

preset_python_version() {
    case "$1" in
        py3.11) echo "3.11" ;;
        py3.12) echo "3.12" ;;
        py3.13) echo "3.13" ;;
        *) die "Unknown preset '$1'. Expected py3.11, py3.12, or py3.13." ;;
    esac
}

build_dir_for_preset() {
    local version
    version="$(preset_python_version "$1")"
    echo "$SOURCE_ROOT/build-orbbec-py$version"
}

validate_sdk_root() {
    local sdk_root="$1"
    [[ -n "$sdk_root" ]] || die "--sdk-root PATH (or ORBBEC_SDK_ROOT) is required."
    [[ -f "$sdk_root/include/libobsensor/ObSensor.hpp" ]] || die "Invalid SDK root '$sdk_root': missing include/libobsensor/ObSensor.hpp"
    [[ -f "$sdk_root/lib/OrbbecSDKConfig.cmake" ]] || die "Invalid SDK root '$sdk_root': missing lib/OrbbecSDKConfig.cmake"
    [[ -f "$sdk_root/lib/libOrbbecSDK.so.2" ]] || die "Invalid SDK root '$sdk_root': missing lib/libOrbbecSDK.so.2"
    [[ -d "$sdk_root/lib/extensions" ]] || die "Invalid SDK root '$sdk_root': missing lib/extensions"
}

resolve_plugin() {
    local preset="$1"
    local requested="${2:-}"
    local candidate
    if [[ -n "$requested" ]]; then
        candidate="$requested"
    elif [[ -n "${ORBBEC_PLUGIN:-}" ]]; then
        candidate="$ORBBEC_PLUGIN"
    elif [[ -x "$SCRIPT_DIR/camera_plugin_orbbec" ]]; then
        candidate="$SCRIPT_DIR/camera_plugin_orbbec"
    else
        candidate="$(build_dir_for_preset "$preset")/src/plugins/orbbec/app/camera_plugin_orbbec"
    fi
    [[ -x "$candidate" ]] || die "Plugin is not executable: $candidate. Run 'build' first or pass --plugin PATH."
    echo "$candidate"
}

resolve_exporter() {
    local preset="$1"
    local requested_plugin="${2:-}"
    local candidate
    # A verify run must never mix a camera plugin and exporter from different build trees.
    if [[ -n "$requested_plugin" ]]; then
        [[ -x "$requested_plugin" ]] || die "Plugin is not executable: $requested_plugin"
        local plugin_dir
        plugin_dir="$(cd "$(dirname "$requested_plugin")" && pwd -P)" \
            || die "Cannot resolve plugin directory: $requested_plugin"
        for candidate in \
                "$plugin_dir/orbbec_mcap_export_media" \
                "$plugin_dir/../export_media/orbbec_mcap_export_media"; do
            if [[ -x "$candidate" ]]; then
                echo "$(cd "$(dirname "$candidate")" && pwd -P)/$(basename "$candidate")"
                return 0
            fi
        done
        die "No embedded-media exporter exists beside plugin or in its build tree: $requested_plugin"
    fi
    if [[ -x "$SCRIPT_DIR/orbbec_mcap_export_media" ]]; then
        candidate="$SCRIPT_DIR/orbbec_mcap_export_media"
    else
        candidate="$(build_dir_for_preset "$preset")/src/plugins/orbbec/export_media/orbbec_mcap_export_media"
    fi
    [[ -x "$candidate" ]] || die "Embedded-media exporter is not executable: $candidate. Run 'build' first."
    echo "$candidate"
}

require_command() {
    local command_name="$1"
    local remediation="$2"
    if ! command -v "$command_name" >/dev/null 2>&1; then
        warn "Missing command '$command_name'. $remediation"
        return 1
    fi
}

require_cmake_version() {
    local version
    require_command cmake "Install CMake 3.24 or newer." || return 1
    version="$(cmake --version | awk 'NR == 1 { print $3 }')"
    if [[ "$(printf '%s\n%s\n' "3.24" "$version" | sort -V | head -n 1)" != "3.24" ]]; then
        warn "CMake $version is too old; Isaac Teleop requires CMake 3.24 or newer."
        return 1
    fi
}

command_doctor() {
    local sdk_root="${ORBBEC_SDK_ROOT:-}"
    local preset="$DEFAULT_PRESET"
    local arg
    while (( $# )); do
        arg="$1"
        case "$arg" in
            --sdk-root) sdk_root="${2:-}"; shift 2 ;;
            --preset) preset="${2:-}"; shift 2 ;;
            --help|-h) usage; return 0 ;;
            *) die "Unknown doctor option: $arg" ;;
        esac
    done
    preset_python_version "$preset" >/dev/null

    local failed=0
    echo "== Orbbec Ego environment diagnosis =="
    echo "Architecture: $(uname -m)"
    if [[ "$(uname -m)" != "x86_64" ]]; then
        warn "The validated host architecture is x86_64."
    fi
    if [[ -r /etc/os-release ]]; then
        . /etc/os-release
        echo "OS: ${PRETTY_NAME:-unknown}"
    fi
    require_command python3 "Install it with: sudo apt update && sudo apt install -y python3" || failed=1
    require_command ffmpeg "Install it with: sudo apt update && sudo apt install -y ffmpeg" || failed=1
    require_command ffprobe "Install it with: sudo apt update && sudo apt install -y ffmpeg" || failed=1
    if command -v python3 >/dev/null 2>&1 \
            && ! python3 -c 'from mcap.reader import make_reader' >/dev/null 2>&1; then
        warn "Python MCAP reader is missing. Install it with: python3 -m pip install --user mcap"
        failed=1
    fi

    local source_checkout=0
    [[ -f "$SOURCE_ROOT/CMakeLists.txt" ]] && source_checkout=1
    if (( source_checkout )); then
        require_cmake_version || failed=1
        require_command c++ "Install it with: sudo apt update && sudo apt install -y build-essential" || failed=1
        require_command uv "Install uv before configuring Isaac Teleop." || failed=1
        if [[ -z "$sdk_root" ]]; then
            warn "No SDK root supplied. Source builds require --sdk-root PATH or ORBBEC_SDK_ROOT."
            failed=1
        fi
    fi

    if [[ -n "$sdk_root" ]]; then
        if validate_sdk_root "$sdk_root"; then
            echo "OrbbecSDK: $sdk_root"
            if [[ -x "$sdk_root/shared/install_udev_rules.sh" ]]; then
                echo "udev rules installer: sudo $sdk_root/shared/install_udev_rules.sh"
            else
                warn "Could not find this SDK release's shared/install_udev_rules.sh. Consult its package documentation."
            fi
        fi
    fi

    local plugin=""
    if plugin="$(resolve_plugin "$preset" "" 2>/dev/null)"; then
        echo "Plugin: $plugin"
    else
        if (( source_checkout )); then
            warn "Plugin is not built yet. Run: $SCRIPT_DIR/orbbec_ego.sh build --sdk-root PATH"
        else
            warn "Plugin is unavailable. Install a complete prebuilt package."
            failed=1
        fi
    fi

    if (( !source_checkout )); then
        local package_file
        for package_file in "$SCRIPT_DIR/orbbec_mcap_export_media" "$SCRIPT_DIR/libOrbbecSDK.so.2" \
                "$SCRIPT_DIR/OrbbecSDKConfig.xml" "$SCRIPT_DIR/extensions"; do
            if [[ ! -e "$package_file" ]]; then
                warn "Prebuilt package is incomplete: missing $package_file"
                failed=1
            fi
        done
        if [[ -n "$plugin" ]] && ! "$plugin" --help >/dev/null 2>&1; then
            warn "Plugin cannot start. Check the packaged SDK libraries and host runtime dependencies."
            failed=1
        fi
    fi

    if command -v lsusb >/dev/null 2>&1; then
        local usb_line bus device node
        usb_line="$(lsusb -d 2bc5: 2>/dev/null || true)"
        if [[ -z "$usb_line" ]]; then
            warn "No Orbbec USB device found. Connect the camera before capabilities or record."
        else
            echo "Detected Orbbec USB device(s):"
            echo "$usb_line"
            while IFS= read -r usb_line; do
                bus="$(awk '{print $2}' <<< "$usb_line")"
                device="$(awk '{gsub(/:/, "", $4); print $4}' <<< "$usb_line")"
                node="/dev/bus/usb/$bus/$device"
                if [[ -r "$node" && -w "$node" ]]; then
                    echo "USB access: OK ($node)"
                else
                    warn "USB access is unavailable for $node. Install this SDK's udev rules, reconnect the camera, then run doctor again."
                fi
            done <<< "$usb_line"
        fi
    else
        warn "lsusb is unavailable; install usbutils to check camera discovery."
    fi

    if (( failed )); then
        return 1
    fi
    echo "Doctor completed. Connect the camera and run capabilities before recording."
}

command_build() {
    local sdk_root="${ORBBEC_SDK_ROOT:-}"
    local preset="$DEFAULT_PRESET"
    local jobs=""
    local clean=0
    local preview=0
    local install_prefix=""
    local -a cmake_args=()
    while (( $# )); do
        case "$1" in
            --sdk-root) sdk_root="${2:-}"; shift 2 ;;
            --preset) preset="${2:-}"; shift 2 ;;
            --jobs) jobs="${2:-}"; shift 2 ;;
            --clean) clean=1; shift ;;
            --preview) preview=1; shift ;;
            --install-prefix) install_prefix="${2:-}"; shift 2 ;;
            --help|-h) usage; return 0 ;;
            *) die "Unknown build option: $1" ;;
        esac
    done
    [[ -f "$SOURCE_ROOT/CMakeLists.txt" ]] || die "The build command is only available from a source checkout."
    preset_python_version "$preset" >/dev/null
    validate_sdk_root "$sdk_root"
    require_cmake_version || die "Install CMake 3.24 or newer, then rerun build."
    local build_dir
    build_dir="$(build_dir_for_preset "$preset")"
    if (( clean )); then
        echo "Removing generated build directory: $build_dir"
        cmake -E rm -rf "$build_dir"
    fi
    local preview_option=OFF
    if (( preview )); then
        preview_option=ON
    fi
    (
        cd "$SOURCE_ROOT"
        cmake -S "$SOURCE_ROOT" -B "$build_dir" \
            -DISAAC_TELEOP_PYTHON_VERSION="$(preset_python_version "$preset")" \
            -DBUILD_VIZ=OFF \
            -DBUILD_PLUGIN_ORBBEC_CAMERA=ON \
            -DORBBEC_SDK_ROOT="$sdk_root" \
            -DORBBEC_ENABLE_PREVIEW="$preview_option"
        local build_args=(--build "$build_dir" --target camera_plugin_orbbec orbbec_mcap_export_media)
        if [[ -n "$jobs" ]]; then
            [[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die "--jobs must be a positive integer."
            build_args+=(--parallel "$jobs")
        else
            build_args+=(--parallel)
        fi
        cmake "${build_args[@]}"
        if [[ -n "$install_prefix" ]]; then
            cmake --install "$build_dir" --prefix "$install_prefix"
        fi
    )
    echo "Build completed."
    echo "Plugin: $build_dir/src/plugins/orbbec/app/camera_plugin_orbbec"
    echo "Next: $SCRIPT_DIR/orbbec_ego.sh capabilities --preset $preset"
}

print_build_provenance() {
    local plugin="$1"
    local sdk_library=""
    echo "== Capture software provenance =="
    echo "Plugin path: $plugin"
    if command -v sha256sum >/dev/null 2>&1; then
        echo "Plugin SHA-256: $(sha256sum "$plugin" | awk '{print $1}')"
    else
        warn "sha256sum is unavailable; the plugin build hash was not recorded."
    fi
    if command -v ldd >/dev/null 2>&1; then
        sdk_library="$({ ldd "$plugin" 2>/dev/null || true; } \
            | awk '$1 ~ /^libOrbbecSDK[.]so/ && $2 == "=>" {print $3; exit}')"
    fi
    if [[ -n "$sdk_library" && -f "$sdk_library" ]]; then
        echo "Orbbec SDK library: $sdk_library"
        if command -v sha256sum >/dev/null 2>&1; then
            echo "Orbbec SDK library SHA-256: $(sha256sum "$sdk_library" | awk '{print $1}')"
        fi
    else
        echo "Orbbec SDK library: unresolved"
        warn "The runtime Orbbec SDK library path could not be resolved with ldd."
    fi
    echo "== Device-advertised capabilities =="
}

emit_capabilities() {
    local plugin="$1"
    local device_uid="${2:-}"
    local -a args=(--list-capabilities)
    [[ -n "$device_uid" ]] && args+=("--device-uid=$device_uid")
    print_build_provenance "$plugin"
    "$plugin" "${args[@]}"
}

command_capabilities() {
    local preset="$DEFAULT_PRESET"
    local plugin_path=""
    local device_uid=""
    while (( $# )); do
        case "$1" in
            --preset) preset="${2:-}"; shift 2 ;;
            --plugin) plugin_path="${2:-}"; shift 2 ;;
            --device-uid) device_uid="${2:-}"; shift 2 ;;
            --help|-h) usage; return 0 ;;
            *) die "Unknown capabilities option: $1" ;;
        esac
    done
    preset_python_version "$preset" >/dev/null
    local plugin
    plugin="$(resolve_plugin "$preset" "$plugin_path")"
    emit_capabilities "$plugin" "$device_uid"
}

record_usage_error() {
    die "record option error: $*. Run '$SCRIPT_DIR/orbbec_ego.sh record --help' for usage."
}

stream_extension() {
    case "$1" in
        mjpg) echo "mjpg" ;;
        h264) echo "h264" ;;
        h265) echo "h265" ;;
        *) die "Unsupported format '$1'." ;;
    esac
}

write_requested_profile_manifest() {
    local output="$1"
    local format="$2"
    local width="$3"
    local height="$4"
    local fps="$5"
    printf '%s\n' \
        '{' \
        '  "version": 1,' \
        '  "streams": {' \
        "    \"ColorLeft\": {\"format\": \"$format\", \"width\": $width, \"height\": $height, \"fps\": $fps}," \
        "    \"ColorRight\": {\"format\": \"$format\", \"width\": $width, \"height\": $height, \"fps\": $fps}" \
        '  }' \
        '}' > "$output"
}

record_config_python() {
    local candidate
    for candidate in python3 python3.13 python3.12 python3.11; do
        if command -v "$candidate" >/dev/null 2>&1 \
                && "$candidate" -c 'import tomllib' >/dev/null 2>&1; then
            command -v "$candidate"
            return 0
        fi
    done
    die "--config requires Python 3.11 or newer for TOML support."
}

read_record_config() {
    local config_file="$1"
    [[ -f "$config_file" ]] || record_usage_error "configuration file does not exist: $config_file"
    local config_python
    config_python="$(record_config_python)"
    "$config_python" - "$config_file" <<'PY'
import sys
import tomllib
from pathlib import Path

path = Path(sys.argv[1])
try:
    with path.open("rb") as stream:
        document = tomllib.load(stream)
except (OSError, tomllib.TOMLDecodeError) as error:
    raise SystemExit(f"Invalid record configuration {path}: {error}") from error

if set(document) != {"record"} or not isinstance(document["record"], dict):
    raise SystemExit(f"{path}: the only top-level entry must be a [record] table")
record = document["record"]
scalar_options = {
    "duration": (int, "--duration"),
    "output": (str, "--output"),
    "format": (str, "--format"),
    "width": (int, "--width"),
    "height": (int, "--height"),
    "fps": (int, "--fps"),
    "device_uid": (str, "--device-uid"),
    "reconnect_timeout": (int, "--reconnect-timeout"),
    "reconnect_interval_ms": (int, "--reconnect-interval-ms"),
    "mcap_media": (str, "--mcap-media"),
    "preset": (str, "--preset"),
    "plugin": (str, "--plugin"),
}
boolean_options = {
    "imu": ("--imu", "--no-imu"),
    "audio": ("--audio", "--no-audio"),
    "preview": ("--preview", "--no-preview"),
    "keep_media_sidecars": ("--keep-media-sidecars", "--no-keep-media-sidecars"),
}
allowed = set(scalar_options) | set(boolean_options) | {"plugin_options"}
unknown = sorted(set(record) - allowed)
if unknown:
    raise SystemExit(f"{path}: unknown [record] option(s): {', '.join(unknown)}")

arguments = []
for name, (expected_type, option) in scalar_options.items():
    if name not in record:
        continue
    value = record[name]
    if type(value) is not expected_type:
        raise SystemExit(f"{path}: record.{name} must be {expected_type.__name__}")
    if isinstance(value, str) and (not value or "\0" in value or "\n" in value):
        raise SystemExit(f"{path}: record.{name} must be a non-empty single-line string")
    arguments.extend((option, str(value)))

for name, (enabled, disabled) in boolean_options.items():
    if name not in record:
        continue
    value = record[name]
    if type(value) is not bool:
        raise SystemExit(f"{path}: record.{name} must be true or false")
    arguments.append(enabled if value else disabled)

plugin_options = record.get("plugin_options", [])
if not isinstance(plugin_options, list) or any(type(value) is not str for value in plugin_options):
    raise SystemExit(f"{path}: record.plugin_options must be an array of strings")
for value in plugin_options:
    if not value.startswith("--") or "\0" in value or "\n" in value:
        raise SystemExit(f"{path}: each record.plugin_options entry must be a single-line --option")
    arguments.extend(("--plugin-option", value))

sys.stdout.buffer.write(b"\0".join(value.encode() for value in arguments))
if arguments:
    sys.stdout.buffer.write(b"\0")
PY
}

command_record() {
    local preset="$DEFAULT_PRESET"
    local plugin_path=""
    local duration=""
    local run_dir=""
    local format="h264"
    local width=1600
    local height=1300
    local fps=30
    local device_uid=""
    local reconnect_timeout=30
    local reconnect_interval_ms=1000
    local request_imu=1
    local request_audio=1
    local preview=0
    local media_mode="metadata-only"
    local keep_sidecars=0
    local -a passthrough=()
    local -a original_args=("$@")
    local config_file=""
    local index
    for (( index = 0; index < ${#original_args[@]}; ++index )); do
        [[ "${original_args[index]}" != "--" ]] || break
        if [[ "${original_args[index]}" == "--config" ]]; then
            (( index + 1 < ${#original_args[@]} )) || record_usage_error "--config requires a FILE.toml"
            [[ -z "$config_file" ]] || record_usage_error "--config may be specified only once"
            config_file="${original_args[index + 1]}"
            (( ++index ))
        fi
    done
    local -a config_args=()
    if [[ -n "$config_file" ]]; then
        local config_args_file
        config_args_file="$(mktemp)"
        if ! read_record_config "$config_file" > "$config_args_file"; then
            rm -f -- "$config_args_file"
            return 1
        fi
        mapfile -d '' -t config_args < "$config_args_file"
        rm -f -- "$config_args_file"
    fi
    set -- "${config_args[@]}" "${original_args[@]}"
    while (( $# )); do
        case "$1" in
            --) shift; passthrough+=("$@"); break ;;
            --config) shift 2 ;;
            --duration) duration="${2:-}"; shift 2 ;;
            --output) run_dir="${2:-}"; shift 2 ;;
            --format) format="${2:-}"; shift 2 ;;
            --width) width="${2:-}"; shift 2 ;;
            --height) height="${2:-}"; shift 2 ;;
            --fps) fps="${2:-}"; shift 2 ;;
            --device-uid) device_uid="${2:-}"; shift 2 ;;
            --reconnect-timeout) reconnect_timeout="${2:-}"; shift 2 ;;
            --reconnect-interval-ms) reconnect_interval_ms="${2:-}"; shift 2 ;;
            --imu) request_imu=1; shift ;;
            --no-imu) request_imu=0; shift ;;
            --audio) request_audio=1; shift ;;
            --no-audio) request_audio=0; shift ;;
            --preview) preview=1; shift ;;
            --no-preview) preview=0; shift ;;
            --mcap-media) media_mode="${2:-}"; shift 2 ;;
            --keep-media-sidecars) keep_sidecars=1; shift ;;
            --no-keep-media-sidecars) keep_sidecars=0; shift ;;
            --preset) preset="${2:-}"; shift 2 ;;
            --plugin) plugin_path="${2:-}"; shift 2 ;;
            --plugin-option) passthrough+=("${2:-}"); shift 2 ;;
            --help|-h) usage; return 0 ;;
            *) record_usage_error "unknown option '$1'" ;;
        esac
    done
    preset_python_version "$preset" >/dev/null
    [[ "$format" == "mjpg" || "$format" == "h264" || "$format" == "h265" ]] || record_usage_error "--format must be mjpg, h264, or h265"
    [[ "$media_mode" == "metadata-only" || "$media_mode" == "embedded" ]] || record_usage_error "--mcap-media must be metadata-only or embedded"
    [[ "$width" =~ ^[1-9][0-9]*$ && "$height" =~ ^[1-9][0-9]*$ && "$fps" =~ ^[1-9][0-9]*$ ]] || record_usage_error "width, height, and fps must be positive integers"
    [[ "$reconnect_timeout" =~ ^[0-9]+$ ]] || record_usage_error "--reconnect-timeout must be a non-negative integer"
    [[ "$reconnect_interval_ms" =~ ^[1-9][0-9]*$ ]] || record_usage_error "--reconnect-interval-ms must be a positive integer"
    if [[ -n "$duration" ]]; then
        [[ "$duration" =~ ^[1-9][0-9]*$ ]] || record_usage_error "--duration must be a positive integer"
    fi
    if (( keep_sidecars )) && [[ "$media_mode" == "metadata-only" ]]; then
        record_usage_error "--keep-media-sidecars is only valid with embedded media"
    fi

    local plugin
    plugin="$(resolve_plugin "$preset" "$plugin_path")"
    local output_base="$SOURCE_ROOT"
    [[ -f "$SOURCE_ROOT/CMakeLists.txt" ]] || output_base="$PWD"
    if [[ -z "$run_dir" ]]; then
        run_dir="$output_base/recordings/orbbec_ego_$(date +%Y%m%d_%H%M%S)"
    elif [[ "$run_dir" != /* ]]; then
        run_dir="$PWD/$run_dir"
    fi
    [[ ! -e "$run_dir" ]] || die "Output directory already exists: $run_dir"
    mkdir -p "$run_dir/logs"
    if [[ "$media_mode" == "metadata-only" ]] || (( keep_sidecars )); then
        mkdir -p "$run_dir/raw"
    fi
    write_requested_profile_manifest "$run_dir/requested_profile.json" "$format" "$width" "$height" "$fps"

    local capabilities_file="$run_dir/capabilities.txt"
    echo "Inspecting connected device before recording..."
    emit_capabilities "$plugin" "$device_uid" | tee "$capabilities_file"
    grep -q '^Sensor LeftColor$' "$capabilities_file" || die "Connected device has no ColorLeft sensor."
    grep -q '^Sensor RightColor$' "$capabilities_file" || die "Connected device has no ColorRight sensor."

    local enable_imu=0
    local enable_audio=0
    if (( request_imu )); then
        if grep -q '^Sensor Accel$' "$capabilities_file" && grep -q '^Sensor Gyro$' "$capabilities_file"; then
            enable_imu=1
        else
            warn "Accel/Gyro are unavailable; continuing without IMU. Use --no-imu to silence this warning."
        fi
    fi
    if (( request_audio )); then
        if grep -q '^Sensor Audio$' "$capabilities_file"; then
            enable_audio=1
        else
            warn "Audio is unavailable; continuing without audio. Use --no-audio to silence this warning."
        fi
    fi

    local extension
    extension="$(stream_extension "$format")"
    local -a plugin_args=(
        "--mcap-filename=$run_dir/metadata.mcap"
        "--mcap-media=$media_mode"
        "--calibration-output=$run_dir/calibration.json"
        "--reconnect-timeout=$reconnect_timeout"
        "--reconnect-interval-ms=$reconnect_interval_ms"
    )
    if [[ "$media_mode" == "metadata-only" ]] || (( keep_sidecars )); then
        plugin_args+=(
            "--add-stream=camera=ColorLeft,output=$run_dir/raw/ColorLeft.$extension,format=$format,width=$width,height=$height,fps=$fps"
            "--add-stream=camera=ColorRight,output=$run_dir/raw/ColorRight.$extension,format=$format,width=$width,height=$height,fps=$fps"
        )
    else
        plugin_args+=(
            "--add-stream=camera=ColorLeft,format=$format,width=$width,height=$height,fps=$fps"
            "--add-stream=camera=ColorRight,format=$format,width=$width,height=$height,fps=$fps"
        )
    fi
    [[ -n "$device_uid" ]] && plugin_args+=("--device-uid=$device_uid")
    (( preview )) && plugin_args+=(--preview)
    (( enable_imu )) && plugin_args+=(--enable-imu --imu-rate=1000)
    if (( enable_audio )); then
        if [[ "$media_mode" == "metadata-only" ]] || (( keep_sidecars )); then
            plugin_args+=("--audio-output=$run_dir/Audio.wav")
        else
            plugin_args+=(--enable-audio)
        fi
    fi
    (( keep_sidecars )) && plugin_args+=(--keep-media-sidecars)
    plugin_args+=("${passthrough[@]}")

    echo "Recording directory: $run_dir"
    echo "Requested exact stereo profile: ${width}x${height}@${fps} ${format}; no profile fallback will be attempted."
    echo "Stop with Ctrl-C once; the plugin will finalize WAV and MCAP files."
    local capture_status
    local -a pipeline_statuses=()
    set +e
    if [[ -n "$duration" ]]; then
        timeout --preserve-status --signal=INT --kill-after=30s "$duration" \
            "$plugin" "${plugin_args[@]}" 2>&1 | tee "$run_dir/logs/capture.log"
    else
        "$plugin" "${plugin_args[@]}" 2>&1 | tee "$run_dir/logs/capture.log"
    fi
    pipeline_statuses=("${PIPESTATUS[@]}")
    capture_status=${pipeline_statuses[0]}
    [[ "${pipeline_statuses[1]}" -eq 0 ]] || die "Unable to write capture log: $run_dir/logs/capture.log"
    set -e
    if [[ "$capture_status" -ne 0 ]]; then
        die "Capture failed with status $capture_status. See $run_dir/logs/capture.log"
    fi
    if [[ -n "$duration" ]]; then
        echo "Timed capture command completed; SIGINT was scheduled after $duration seconds."
    fi
    local -a verify_args=("$run_dir" --preset "$preset" --plugin "$plugin" --media-mode "$media_mode")
    (( enable_imu )) && verify_args+=(--expect-imu)
    (( enable_audio )) && verify_args+=(--expect-audio)
    command_verify "${verify_args[@]}"
}

find_media_file() {
    local directory="$1"
    local stream="$2"
    local candidate
    for candidate in "$directory/$stream.h264" "$directory/$stream.h265" "$directory/$stream.mjpg"; do
        if [[ -s "$candidate" ]]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

check_capture_health_log() {
    local capture_log="$1"
    if [[ ! -s "$capture_log" ]]; then
        warn "No capture log is available; sequence-gap and queue-drop checks were skipped."
        return 0
    fi
    if grep -Eq '[[:space:]][1-9][0-9]* sequence gaps' "$capture_log"; then
        die "Capture log reports a video sequence gap: $capture_log"
    fi
    if grep -Eq '(^|[[:space:]])dropped=[1-9][0-9]*([[:space:]]|$)' "$capture_log"; then
        die "Capture log reports dropped metadata events: $capture_log"
    fi
    if grep -Eq 'video_frame_sets_dropped=[1-9][0-9]*([[:space:]]|$)' "$capture_log"; then
        die "Capture log reports dropped video frame sets: $capture_log"
    fi
    if grep -Eq 'sequence gaps|video_frame_sets_dropped=' "$capture_log"; then
        echo "Capture log sequence-gap and queue-drop counters: OK"
    else
        warn "Capture log contains no statistics counters; inspect it before accepting the run."
    fi
}

video_signature() {
    local media_file="$1"
    local -a input_args=()
    [[ "$media_file" == *.mjpg ]] && input_args=(-f mjpeg)
    ffprobe -v error "${input_args[@]}" -select_streams v:0 \
        -show_entries stream=codec_name,width,height,r_frame_rate \
        -of csv=p=0 "$media_file"
}

video_frame_count() {
    local media_file="$1"
    local -a input_args=()
    [[ "$media_file" == *.mjpg ]] && input_args=(-f mjpeg)
    ffprobe -v error "${input_args[@]}" -count_frames -select_streams v:0 \
        -show_entries stream=nb_read_frames -of default=noprint_wrappers=1:nokey=1 "$media_file"
}

decode_video() {
    local stream_name="$1"
    local media_file="$2"
    local -a input_args=()
    [[ "$media_file" == *.mjpg ]] && input_args=(-f mjpeg)
    ffmpeg -v error -xerror -err_detect explode "${input_args[@]}" -i "$media_file" -f null -
    echo "$stream_name complete decode: OK"
}

verify_requested_profile_manifest() {
    local manifest="$1"
    local left_signature="$2"
    local right_signature="$3"
    python3 - "$manifest" "$left_signature" "$right_signature" <<'PY'
from fractions import Fraction
import json
from pathlib import Path
import sys

manifest_path = Path(sys.argv[1])
actual_signatures = {
    "ColorLeft": sys.argv[2],
    "ColorRight": sys.argv[3],
}
try:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as error:
    raise SystemExit(f"Invalid requested profile manifest {manifest_path}: {error}") from error

if manifest.get("version") != 1:
    raise SystemExit(f"Unsupported requested profile manifest version in {manifest_path}")
streams = manifest.get("streams")
expected_names = set(actual_signatures)
if not isinstance(streams, dict) or set(streams) != expected_names:
    raise SystemExit(f"Requested profile manifest must contain exactly {sorted(expected_names)}")

codec_names = {"mjpg": "mjpeg", "h264": "h264", "h265": "hevc"}
for stream_name, signature in actual_signatures.items():
    requested = streams[stream_name]
    if not isinstance(requested, dict):
        raise SystemExit(f"Invalid requested profile for {stream_name}")
    requested_format = requested.get("format")
    if requested_format not in codec_names:
        raise SystemExit(f"Invalid requested format for {stream_name}: {requested_format!r}")
    for field in ("width", "height", "fps"):
        if type(requested.get(field)) is not int or requested[field] <= 0:
            raise SystemExit(f"Invalid requested {field} for {stream_name}: {requested.get(field)!r}")
    parts = signature.split(",")
    if len(parts) != 4:
        raise SystemExit(f"Invalid ffprobe signature for {stream_name}: {signature!r}")
    codec, width, height, rate = parts
    try:
        actual_width = int(width)
        actual_height = int(height)
        actual_rate = Fraction(rate)
    except (ValueError, ZeroDivisionError) as error:
        raise SystemExit(f"Invalid ffprobe signature for {stream_name}: {signature!r}") from error
    expected_media = (codec_names[requested_format], requested["width"], requested["height"])
    actual_media = (codec, actual_width, actual_height)
    if actual_media != expected_media:
        raise SystemExit(
            f"{stream_name} recorded profile {signature!r} does not match request "
            f"{requested_format},{requested['width']},{requested['height']},{requested['fps']}/1"
        )
    # Raw MJPEG carries no frame-rate declaration; ffprobe reports its demuxer
    # default. Its actual FPS is checked from MCAP metadata and device timestamps.
    if requested_format != "mjpg" and actual_rate != Fraction(requested["fps"], 1):
        raise SystemExit(
            f"{stream_name} recorded rate {actual_rate} does not match requested {requested['fps']}/1"
        )
print("Requested codec/resolution and encoded-video rate match ColorLeft and ColorRight: OK")
PY
}

verify_mcap_video_timing() {
    local mcap="$1"
    local mode="$2"
    local left_frames="$3"
    local right_frames="$4"
    local manifest="$5"
    python3 - "$mcap" "$mode" "$left_frames" "$right_frames" "$manifest" <<'PY'
import json
from pathlib import Path
import struct
import sys

from mcap.reader import make_reader


def unpack_from(fmt, data, offset, label):
    size = struct.calcsize(fmt)
    if offset < 0 or offset + size > len(data):
        raise SystemExit(f"Invalid FlatBuffer while reading {label} at byte {offset}")
    return struct.unpack_from(fmt, data, offset)


def field_position(data, table, field_id):
    (vtable_delta,) = unpack_from("<i", data, table, "vtable offset")
    vtable = table - vtable_delta
    (vtable_size,) = unpack_from("<H", data, vtable, "vtable size")
    entry = vtable + 4 + 2 * field_id
    if entry + 2 > vtable + vtable_size:
        return None
    (field_offset,) = unpack_from("<H", data, entry, f"field {field_id}")
    return None if field_offset == 0 else table + field_offset


def scalar_field(data, table, field_id, fmt, default):
    position = field_position(data, table, field_id)
    return default if position is None else unpack_from(fmt, data, position, f"field {field_id}")[0]


def video_record(data, embedded_media):
    # Metadata and embedded-video roots share field ids for data and timestamp;
    # their data tables share sequence/width/height/fps/format ids 1 through 5.
    (record_table,) = unpack_from("<I", data, 0, "root table")
    data_position = field_position(data, record_table, 0)
    timestamp_position = field_position(data, record_table, 1)
    if data_position is None or timestamp_position is None:
        raise SystemExit("Orbbec video record is missing data or DeviceDataTimestamp")
    (data_offset,) = unpack_from("<I", data, data_position, "video data table")
    data_table = data_position + data_offset
    timestamp = unpack_from("<qqq", data, timestamp_position, "DeviceDataTimestamp")[2]
    sequence = scalar_field(data, data_table, 1, "<Q", 0)
    width = scalar_field(data, data_table, 2, "<I", 0)
    height = scalar_field(data, data_table, 3, "<I", 0)
    fps = scalar_field(data, data_table, 4, "<I", 0)
    pixel_format = scalar_field(data, data_table, 5, "<B", 0)
    capture_epoch = scalar_field(data, data_table, 7 if embedded_media else 8, "<I", 0)
    return timestamp, sequence, (pixel_format, width, height, fps), capture_epoch


path, mode = sys.argv[1:3]
expected_counts = {
    "ColorLeft": int(sys.argv[3]),
    "ColorRight": int(sys.argv[4]),
}
manifest_path = Path(sys.argv[5])
requested_profiles = {}
if manifest_path.exists():
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        format_ids = {"mjpg": 0, "h264": 1, "h265": 2}
        for stream in expected_counts:
            requested = manifest["streams"][stream]
            requested_profiles[stream] = (
                format_ids[requested["format"]],
                requested["width"],
                requested["height"],
                requested["fps"],
            )
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise SystemExit(f"Invalid requested profile manifest {manifest_path}: {error}") from error

prefixes = ["orbbec_metadata"]
if mode == "embedded":
    prefixes.append("orbbec_media")
topics = {
    f"{prefix}/{stream}": {
        "stream": stream,
        "count": 0,
        "first": None,
        "last": None,
        "first_sequence": None,
        "last_sequence": None,
        "profile": None,
        "epochs": {},
        "timeline": [],
    }
    for prefix in prefixes
    for stream in expected_counts
}
with open(path, "rb") as stream:
    for _schema, channel, message in make_reader(stream).iter_messages(log_time_order=False):
        state = topics.get(channel.topic)
        if state is None:
            continue
        timestamp, sequence, profile, capture_epoch = video_record(
            message.data, channel.topic.startswith("orbbec_media/")
        )
        epoch = state["epochs"].setdefault(capture_epoch, {
            "count": 0, "first": None, "last": None,
            "first_sequence": None, "last_sequence": None,
        })
        if epoch["last"] is not None and timestamp <= epoch["last"]:
            raise SystemExit(
                f"Non-increasing raw device timestamp on {channel.topic} epoch {capture_epoch}: "
                f"previous={epoch['last']} current={timestamp}"
            )
        if epoch["first"] is None:
            epoch["first"] = timestamp
            epoch["first_sequence"] = sequence
        if epoch["last_sequence"] is not None:
            if sequence <= epoch["last_sequence"]:
                raise SystemExit(
                    f"Non-increasing sequence on {channel.topic} epoch {capture_epoch}: "
                    f"previous={epoch['last_sequence']} current={sequence}"
                )
            if sequence != epoch["last_sequence"] + 1:
                raise SystemExit(
                    f"Sequence gap on {channel.topic} epoch {capture_epoch}: "
                    f"previous={epoch['last_sequence']} current={sequence}"
                )
        if state["profile"] is not None and profile != state["profile"]:
            raise SystemExit(f"Profile changed on {channel.topic}: {state['profile']} -> {profile}")
        state["last"] = timestamp
        state["last_sequence"] = sequence
        state["profile"] = profile
        state["count"] += 1
        state["timeline"].append((capture_epoch, sequence, timestamp, profile))
        epoch["last"] = timestamp
        epoch["last_sequence"] = sequence
        epoch["count"] += 1

for topic, state in topics.items():
    stream = state["stream"]
    if state["count"] != expected_counts[stream]:
        raise SystemExit(
            f"{topic} count {state['count']} differs from decoded {stream} count {expected_counts[stream]}"
        )
    if stream in requested_profiles and state["profile"] != requested_profiles[stream]:
        raise SystemExit(
            f"{topic} MCAP profile {state['profile']} differs from requested {requested_profiles[stream]}"
        )
    for capture_epoch, epoch in sorted(state["epochs"].items()):
        if epoch["count"] < 2 or epoch["last"] <= epoch["first"]:
            raise SystemExit(f"{topic} epoch {capture_epoch} has too few valid raw device timestamps")
        declared_fps = state["profile"][3]
        if declared_fps <= 0:
            raise SystemExit(f"{topic} has invalid declared FPS {declared_fps}")
        observed_fps = (epoch["count"] - 1) * 1_000_000_000 / (epoch["last"] - epoch["first"])
        tolerance = max(0.5, declared_fps * 0.05)
        if abs(observed_fps - declared_fps) > tolerance:
            raise SystemExit(
                f"{topic} epoch {capture_epoch} raw-device cadence is {observed_fps:.3f} FPS; "
                f"the MCAP profile declares {declared_fps} FPS"
            )
        print(
            f"  {topic}: epoch={capture_epoch} count={epoch['count']} profile={state['profile']} "
            f"raw_device_fps={observed_fps:.3f} span_ns={epoch['last'] - epoch['first']}"
        )
if mode == "embedded":
    for stream in expected_counts:
        metadata = topics[f"orbbec_metadata/{stream}"]
        media = topics[f"orbbec_media/{stream}"]
        comparable = ("count", "profile", "timeline")
        if any(metadata[field] != media[field] for field in comparable):
            raise SystemExit(f"Embedded and metadata timelines differ for {stream}")
print("MCAP video counts and raw device timestamp cadence: OK")
PY
}

command_verify_impl() {
    local run_dir="${1:-}"
    shift || true
    [[ -n "$run_dir" ]] || die "verify requires a RUN_DIRECTORY"
    [[ -d "$run_dir" ]] || die "Run directory does not exist: $run_dir"
    local preset="$DEFAULT_PRESET"
    local plugin_path=""
    local expected_mode=""
    local expect_imu=0
    local expect_audio=0
    while (( $# )); do
        case "$1" in
            --preset) preset="${2:-}"; shift 2 ;;
            --plugin) plugin_path="${2:-}"; shift 2 ;;
            --media-mode) expected_mode="${2:-}"; shift 2 ;;
            --expect-imu) expect_imu=1; shift ;;
            --expect-audio) expect_audio=1; shift ;;
            --help|-h) usage; return 0 ;;
            *) die "Unknown verify option: $1" ;;
        esac
    done
    preset_python_version "$preset" >/dev/null
    run_dir="$(cd "$run_dir" && pwd)"
    local mcap="$run_dir/metadata.mcap"
    [[ -s "$mcap" ]] || die "Missing or empty MCAP: $mcap"
    local partial_file
    partial_file="$(find "$run_dir" -name '*.partial' -print -quit)"
    [[ -z "$partial_file" ]] || die "Incomplete capture: found $partial_file"
    require_command ffmpeg "Install it with: sudo apt update && sudo apt install -y ffmpeg" || exit 1
    require_command ffprobe "Install it with: sudo apt update && sudo apt install -y ffmpeg" || exit 1
    require_command python3 "Install Python 3 and the standard MCAP reader: python3 -m pip install --user mcap" || exit 1
    python3 -c 'from mcap.reader import make_reader' 2>/dev/null || die "Python MCAP reader is missing. Install it with: python3 -m pip install --user mcap"

    local mode="$expected_mode"
    if [[ -z "$mode" ]]; then
        mode="$(python3 - "$mcap" <<'PY'
import sys
from mcap.reader import make_reader
topics = set()
with open(sys.argv[1], "rb") as stream:
    for _, channel, _ in make_reader(stream).iter_messages(log_time_order=False):
        topics.add(channel.topic)
print("embedded" if any(topic.startswith("orbbec_media/") for topic in topics) else "metadata-only")
PY
        )"
    fi
    [[ "$mode" == "metadata-only" || "$mode" == "embedded" ]] || die "Unknown MCAP media mode '$mode'"

    local -a topic_check_args=("$mcap" "$mode")
    (( expect_imu )) && topic_check_args+=(--expect-imu)
    (( expect_audio )) && topic_check_args+=(--expect-audio)
    python3 - "${topic_check_args[@]}" <<'PY'
from collections import Counter
import struct
import sys
from mcap.reader import make_reader


def unpack_from(fmt, data, offset, label):
    size = struct.calcsize(fmt)
    if offset < 0 or offset + size > len(data):
        raise SystemExit(f"Invalid FlatBuffer while reading {label} at byte {offset}")
    return struct.unpack_from(fmt, data, offset)


def field_position(data, table, field_id):
    (vtable_delta,) = unpack_from("<i", data, table, "vtable offset")
    vtable = table - vtable_delta
    (vtable_size,) = unpack_from("<H", data, vtable, "vtable size")
    entry = vtable + 4 + 2 * field_id
    if entry + 2 > vtable + vtable_size:
        return None
    (field_offset,) = unpack_from("<H", data, entry, f"field {field_id}")
    return None if field_offset == 0 else table + field_offset


def scalar_field(data, table, field_id, fmt, default=0):
    position = field_position(data, table, field_id)
    return default if position is None else unpack_from(fmt, data, position, f"field {field_id}")[0]


def record_data(data, label):
    (record_table,) = unpack_from("<I", data, 0, f"{label} root table")
    data_position = field_position(data, record_table, 0)
    timestamp_position = field_position(data, record_table, 1)
    if data_position is None or timestamp_position is None:
        raise SystemExit(f"{label} is missing data or DeviceDataTimestamp")
    (data_offset,) = unpack_from("<I", data, data_position, f"{label} data table")
    table = data_position + data_offset
    device_timestamp = unpack_from("<qqq", data, timestamp_position, f"{label} timestamp")[2]
    return table, device_timestamp


def vector_field(data, table, field_id, element_size, label):
    position = field_position(data, table, field_id)
    if position is None:
        return 0, 0
    (offset,) = unpack_from("<I", data, position, label)
    vector = position + offset
    (count,) = unpack_from("<I", data, vector, f"{label} length")
    begin = vector + 4
    if begin + count * element_size > len(data):
        raise SystemExit(f"Invalid FlatBuffer while reading {label} elements")
    return count, begin


path, mode = sys.argv[1:3]
expect_imu = "--expect-imu" in sys.argv
expect_audio = "--expect-audio" in sys.argv
counts = Counter()
last_log_time = {}
non_monotonic = []
video_epochs = {}
imu = {
    "orbbec_imu/Accel": {"sensor": 0, "batches": 0, "samples": 0, "epochs": {}},
    "orbbec_imu/Gyro": {"sensor": 1, "batches": 0, "samples": 0, "epochs": {}},
}
audio_topics = ["orbbec_audio/Audio"]
if mode == "embedded":
    audio_topics.append("orbbec_media/Audio")
audio = {topic: {"chunks": 0, "samples": 0, "timeline": [], "epochs": {}} for topic in audio_topics}

with open(path, "rb") as stream:
    for _schema, channel, message in make_reader(stream).iter_messages(log_time_order=False):
        topic = channel.topic
        counts[topic] += 1
        previous = last_log_time.get(topic)
        if previous is not None and message.log_time < previous:
            non_monotonic.append(topic)
        last_log_time[topic] = message.log_time

        if topic == "orbbec_metadata/ColorLeft":
            table, timestamp = record_data(message.data, topic)
            capture_epoch = scalar_field(message.data, table, 8, "<I")
            epoch = video_epochs.setdefault(capture_epoch, {"first": timestamp, "last": timestamp})
            epoch["last"] = timestamp

        if topic in imu:
            table, record_timestamp = record_data(message.data, topic)
            state = imu[topic]
            sensor = scalar_field(message.data, table, 0, "<b")
            sequence = scalar_field(message.data, table, 1, "<Q")
            sample_rate = scalar_field(message.data, table, 2, "<I")
            capture_epoch = scalar_field(message.data, table, 5, "<I")
            epoch = state["epochs"].setdefault(
                capture_epoch, {"batches": 0, "samples": 0, "last_sequence": None, "first": None, "last": None}
            )
            sample_count, samples = vector_field(message.data, table, 4, 48, f"{topic} samples")
            if sensor != state["sensor"]:
                raise SystemExit(f"{topic} contains sensor id {sensor}")
            if sample_rate <= 0:
                raise SystemExit(f"{topic} has invalid sample_rate_hz {sample_rate}")
            if sample_count == 0:
                raise SystemExit(f"{topic} contains an empty IMU batch")
            if state.get("rate") not in (None, sample_rate):
                raise SystemExit(f"{topic} sample_rate_hz changed from {state['rate']} to {sample_rate}")
            if epoch["last_sequence"] is not None and sequence != epoch["last_sequence"] + 1:
                raise SystemExit(
                    f"{topic} epoch {capture_epoch} batch sequence is not contiguous: "
                    f"previous={epoch['last_sequence']} current={sequence}"
                )
            batch_timestamps = [
                unpack_from("<q", message.data, samples + index * 48 + 40, f"{topic} sample timestamp")[0]
                for index in range(sample_count)
            ]
            previous_sample = epoch["last"]
            for timestamp in batch_timestamps:
                if previous_sample is not None and timestamp <= previous_sample:
                    raise SystemExit(
                        f"{topic} raw device sample timestamp is not increasing: "
                        f"previous={previous_sample} current={timestamp}"
                    )
                previous_sample = timestamp
            if record_timestamp != batch_timestamps[-1]:
                raise SystemExit(
                    f"{topic} batch timestamp {record_timestamp} does not match its last sample "
                    f"{batch_timestamps[-1]}"
                )
            if epoch["first"] is None:
                epoch["first"] = batch_timestamps[0]
            epoch["last"] = batch_timestamps[-1]
            epoch["last_sequence"] = sequence
            epoch["batches"] += 1
            epoch["samples"] += sample_count
            state["rate"] = sample_rate
            state["batches"] += 1
            state["samples"] += sample_count

        if topic in audio:
            table, timestamp = record_data(message.data, topic)
            state = audio[topic]
            sequence = scalar_field(message.data, table, 0, "<Q")
            sample_rate = scalar_field(message.data, table, 1, "<I")
            channels = scalar_field(message.data, table, 2, "<H")
            bits = scalar_field(message.data, table, 3, "<H")
            sample_format = scalar_field(message.data, table, 4, "<b")
            sample_count = scalar_field(message.data, table, 5, "<I")
            capture_epoch = scalar_field(message.data, table, 7 if topic == "orbbec_media/Audio" else 8, "<I")
            epoch = state["epochs"].setdefault(
                capture_epoch,
                {"chunks": 0, "samples": 0, "first": None, "last": None,
                 "last_sequence": None, "first_sample_count": None, "last_sample_count": None},
            )
            profile = (sample_rate, channels, bits, sample_format)
            if profile != (48_000, 1, 16, 0):
                raise SystemExit(f"{topic} has unsupported audio profile {profile}; expected (48000, 1, 16, S16LE)")
            if sample_count == 0:
                raise SystemExit(f"{topic} contains an empty audio chunk")
            expected_bytes = sample_count * channels * (bits // 8)
            if topic == "orbbec_media/Audio":
                pcm_bytes, _pcm = vector_field(message.data, table, 6, 1, f"{topic} PCM data")
                if pcm_bytes != expected_bytes:
                    raise SystemExit(
                        f"{topic} PCM byte count {pcm_bytes} does not match sample_count/profile {expected_bytes}"
                    )
            else:
                byte_count = scalar_field(message.data, table, 7, "<I")
                if byte_count != expected_bytes:
                    raise SystemExit(
                        f"{topic} byte_count {byte_count} does not match sample_count/profile {expected_bytes}"
                    )
            if state.get("profile") not in (None, profile):
                raise SystemExit(f"{topic} audio profile changed from {state['profile']} to {profile}")
            if epoch["last_sequence"] is not None and sequence != epoch["last_sequence"] + 1:
                raise SystemExit(
                    f"{topic} epoch {capture_epoch} chunk sequence is not contiguous: "
                    f"previous={epoch['last_sequence']} current={sequence}"
                )
            if epoch["last"] is not None and timestamp <= epoch["last"]:
                raise SystemExit(
                    f"{topic} epoch {capture_epoch} raw device chunk timestamp is not increasing: "
                    f"previous={epoch['last']} current={timestamp}"
                )
            if epoch["first"] is None:
                epoch["first"] = timestamp
                epoch["first_sample_count"] = sample_count
            epoch["last"] = timestamp
            epoch["last_sample_count"] = sample_count
            epoch["last_sequence"] = sequence
            epoch["chunks"] += 1
            epoch["samples"] += sample_count
            state["profile"] = profile
            state["chunks"] += 1
            state["samples"] += sample_count
            state["timeline"].append((capture_epoch, sequence, timestamp, sample_count, profile))

expect_imu = expect_imu or any(counts[topic] for topic in imu)
expect_audio = expect_audio or any(counts[topic] for topic in audio)

required = [
    "orbbec_metadata/ColorLeft", "orbbec_metadata/ColorRight",
    "orbbec_calibration/Calibration", "orbbec_device/DeviceState",
]
if mode == "embedded":
    required += ["orbbec_media/ColorLeft", "orbbec_media/ColorRight"]
if expect_imu:
    required += ["orbbec_imu/Accel", "orbbec_imu/Gyro"]
if expect_audio:
    required += ["orbbec_audio/Audio"]
    if mode == "embedded":
        required += ["orbbec_media/Audio"]
missing = [topic for topic in required if counts[topic] == 0]
for topic in sorted(counts):
    print(f"  {topic}: {counts[topic]}")
if missing:
    raise SystemExit(f"Missing or empty required MCAP topics: {missing}")
if non_monotonic:
    raise SystemExit(f"Non-monotonic MCAP log time on topics: {sorted(set(non_monotonic))}")

if expect_imu or expect_audio:
    for capture_epoch, epoch in video_epochs.items():
        if epoch["last"] <= epoch["first"]:
            raise SystemExit(f"ColorLeft epoch {capture_epoch} has too few timestamps for auxiliary coverage")

if expect_imu:
    for topic, state in imu.items():
        if set(state["epochs"]) != set(video_epochs):
            raise SystemExit(f"{topic} capture epochs differ from video: {sorted(state['epochs'])}")
        for capture_epoch, epoch in sorted(state["epochs"].items()):
            if epoch["samples"] < 2 or epoch["last"] <= epoch["first"]:
                raise SystemExit(f"{topic} epoch {capture_epoch} has too few samples")
            video_span = video_epochs[capture_epoch]["last"] - video_epochs[capture_epoch]["first"]
            sample_span = epoch["last"] - epoch["first"]
            observed_rate = (epoch["samples"] - 1) * 1_000_000_000 / sample_span
            tolerance = max(5.0, state["rate"] * 0.05)
            if abs(observed_rate - state["rate"]) > tolerance:
                raise SystemExit(
                    f"{topic} epoch {capture_epoch} raw-device cadence is {observed_rate:.3f} Hz; "
                    f"batches declare {state['rate']} Hz"
                )
            coverage = sample_span + 1_000_000_000 / state["rate"]
            if coverage < video_span * 0.90:
                raise SystemExit(
                    f"{topic} epoch {capture_epoch} covers only {coverage / 1_000_000_000:.3f}s "
                    f"of the {video_span / 1_000_000_000:.3f}s video timeline"
                )
            print(
                f"  {topic}: epoch={capture_epoch} batches={epoch['batches']} samples={epoch['samples']} "
                f"declared_hz={state['rate']} raw_device_hz={observed_rate:.3f}"
            )
    if imu["orbbec_imu/Accel"]["rate"] != imu["orbbec_imu/Gyro"]["rate"]:
        raise SystemExit("Accel and Gyro declare different sample rates")

if expect_audio:
    for topic, state in audio.items():
        sample_rate = state["profile"][0]
        if set(state["epochs"]) != set(video_epochs):
            raise SystemExit(f"{topic} capture epochs differ from video: {sorted(state['epochs'])}")
        for capture_epoch, epoch in sorted(state["epochs"].items()):
            if epoch["chunks"] < 2 or epoch["last"] <= epoch["first"]:
                raise SystemExit(f"{topic} epoch {capture_epoch} has too few chunks")
            video_span = video_epochs[capture_epoch]["last"] - video_epochs[capture_epoch]["first"]
            timestamp_span = epoch["last"] - epoch["first"]
            timestamped_counts = (
                epoch["samples"] - epoch["first_sample_count"],
                epoch["samples"] - epoch["last_sample_count"],
            )
            observed_rates = [count * 1_000_000_000 / timestamp_span for count in timestamped_counts]
            observed_rate = min(observed_rates, key=lambda rate: abs(rate - sample_rate))
            tolerance = max(5.0, sample_rate * 0.05)
            if abs(observed_rate - sample_rate) > tolerance:
                raise SystemExit(
                    f"{topic} epoch {capture_epoch} raw-device cadence is {observed_rate:.3f} Hz; "
                    f"chunks declare {sample_rate} Hz"
                )
            duration = epoch["samples"] * 1_000_000_000 / sample_rate
            if duration < video_span * 0.90:
                raise SystemExit(
                    f"{topic} epoch {capture_epoch} contains only {duration / 1_000_000_000:.3f}s "
                    f"of PCM for the {video_span / 1_000_000_000:.3f}s video timeline"
                )
            print(
                f"  {topic}: epoch={capture_epoch} chunks={epoch['chunks']} samples={epoch['samples']} "
                f"duration_s={duration / 1_000_000_000:.3f} raw_device_hz={observed_rate:.3f}"
            )
    if mode == "embedded":
        metadata = audio["orbbec_audio/Audio"]
        media = audio["orbbec_media/Audio"]
        if metadata["timeline"] != media["timeline"]:
            raise SystemExit("Embedded PCM and audio-metadata timelines differ")
print("MCAP Footer, required topics, and per-topic log-time monotonicity: OK")
PY

    check_capture_health_log "$run_dir/logs/capture.log"

    local media_dir="$run_dir/raw"
    local retained_media_dir=""
    local temporary_export=""
    if [[ "$mode" == "embedded" ]]; then
        [[ -d "$media_dir" ]] && retained_media_dir="$media_dir"
        temporary_export="$(mktemp -d "$run_dir/.verify_export.XXXXXX")"
        # Verification failures exit from deep helper calls; EXIT cleanup prevents a
        # multi-gigabyte embedded export from being stranded beside the recording.
        local cleanup_command
        printf -v cleanup_command 'rm -rf -- %q' "$temporary_export"
        trap "$cleanup_command" EXIT
        local exporter
        exporter="$(resolve_exporter "$preset" "$plugin_path")" || return 1
        "$exporter" "$mcap" "$temporary_export"
        media_dir="$temporary_export"
    fi
    local left right
    left="$(find_media_file "$media_dir" ColorLeft)" || die "Could not find exported ColorLeft media in $media_dir"
    right="$(find_media_file "$media_dir" ColorRight)" || die "Could not find exported ColorRight media in $media_dir"
    if [[ -n "$retained_media_dir" ]]; then
        local retained_left retained_right
        retained_left="$(find_media_file "$retained_media_dir" ColorLeft)" \
            || die "Could not find retained ColorLeft media in $retained_media_dir"
        retained_right="$(find_media_file "$retained_media_dir" ColorRight)" \
            || die "Could not find retained ColorRight media in $retained_media_dir"
        cmp -s "$left" "$retained_left" || die "Embedded and retained ColorLeft media differ"
        cmp -s "$right" "$retained_right" || die "Embedded and retained ColorRight media differ"
        echo "Embedded video matches retained media sidecars: OK"
    fi
    decode_video ColorLeft "$left"
    decode_video ColorRight "$right"
    local left_signature right_signature left_frames right_frames
    left_signature="$(video_signature "$left")"
    right_signature="$(video_signature "$right")"
    [[ -n "$left_signature" ]] || die "ffprobe found no ColorLeft video stream: $left"
    [[ -n "$right_signature" ]] || die "ffprobe found no ColorRight video stream: $right"
    [[ "$left_signature" == "$right_signature" ]] \
        || die "Stereo media profiles differ: ColorLeft=$left_signature ColorRight=$right_signature"
    local requested_profile_manifest="$run_dir/requested_profile.json"
    if [[ -e "$requested_profile_manifest" ]]; then
        verify_requested_profile_manifest "$requested_profile_manifest" "$left_signature" "$right_signature"
    else
        warn "No requested_profile.json exists; exact requested-versus-recorded profile checks were skipped for this legacy run."
    fi
    left_frames="$(video_frame_count "$left")"
    right_frames="$(video_frame_count "$right")"
    [[ "$left_frames" =~ ^[1-9][0-9]*$ ]] || die "Could not count decoded ColorLeft frames: $left_frames"
    [[ "$right_frames" =~ ^[1-9][0-9]*$ ]] || die "Could not count decoded ColorRight frames: $right_frames"
    echo "Stereo media profile (codec,width,height,fps): $left_signature"
    echo "Decoded frames: ColorLeft=$left_frames ColorRight=$right_frames"
    if [[ "$left_frames" != "$right_frames" ]]; then
        warn "Stereo decoded counts differ at their initial IDR boundaries; per-stream MCAP counts and cadence must still pass."
    fi
    verify_mcap_video_timing \
        "$mcap" "$mode" "$left_frames" "$right_frames" "$requested_profile_manifest"
    local wav=""
    if [[ "$mode" == "embedded" ]]; then
        wav="$media_dir/Audio.wav"
        if [[ -s "$run_dir/Audio.wav" ]]; then
            [[ -s "$wav" ]] || die "Embedded MCAP is missing audio retained as $run_dir/Audio.wav"
            cmp -s "$wav" "$run_dir/Audio.wav" || die "Embedded and retained PCM audio differ"
            echo "Embedded audio matches retained WAV sidecar: OK"
        fi
    else
        wav="$run_dir/Audio.wav"
    fi
    if [[ -s "$wav" ]]; then
        local wav_signature
        wav_signature="$(ffprobe -v error \
            -show_entries stream=codec_name,sample_rate,channels,bits_per_sample \
            -of default=noprint_wrappers=1 "$wav")"
        echo "$wav_signature"
        if (( expect_audio )); then
            grep -Fx 'codec_name=pcm_s16le' <<< "$wav_signature" >/dev/null \
                || die "Expected PCM S16_LE audio in $wav"
            grep -Fx 'sample_rate=48000' <<< "$wav_signature" >/dev/null \
                || die "Expected 48000 Hz audio in $wav"
            grep -Fx 'channels=1' <<< "$wav_signature" >/dev/null \
                || die "Expected mono audio in $wav"
            grep -Fx 'bits_per_sample=16' <<< "$wav_signature" >/dev/null \
                || die "Expected 16-bit audio in $wav"
        fi
    elif (( expect_audio )); then
        die "Audio was requested but no WAV sidecar or exported embedded audio exists."
    else
        warn "No WAV sidecar exists. This is expected only when audio was unavailable, disabled, or embedded without sidecars."
    fi
    if [[ -n "$temporary_export" ]]; then
        rm -rf -- "$temporary_export"
        temporary_export=""
        trap - EXIT
    fi
    echo "Delivery file: $mcap"
    if command -v sha256sum >/dev/null 2>&1; then
        echo "Delivery SHA-256: $(sha256sum "$mcap" | awk '{print $1}')"
        echo "ColorLeft SHA-256: $(sha256sum "$left" | awk '{print $1}')"
        echo "ColorRight SHA-256: $(sha256sum "$right" | awk '{print $1}')"
        [[ ! -s "$wav" ]] || echo "Audio SHA-256: $(sha256sum "$wav" | awk '{print $1}')"
    else
        warn "sha256sum is unavailable; media fingerprints were not added to the report."
    fi
    echo "Verification passed: $run_dir"
}

command_verify() {
    local run_dir="${1:-}"
    if [[ ! -d "$run_dir" ]]; then
        command_verify_impl "$@"
        return
    fi

    run_dir="$(cd "$run_dir" && pwd)"
    local report="$run_dir/verification_report.txt"
    local failed_report="$run_dir/verification_report.failed.txt"
    local working_report="$run_dir/.verification_report.txt.inprogress"
    local -a pipeline_status

    set +e
    {
        set -e
        echo "Orbbec MCAP consistency verification report"
        echo "Generated (UTC): $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "Run directory: $run_dir"
        echo
        command_verify_impl "$run_dir" "${@:2}"
    } 2>&1 | tee "$working_report"
    pipeline_status=("${PIPESTATUS[@]}")
    set -e

    if (( pipeline_status[1] != 0 )); then
        die "Could not write consistency report: $working_report"
    fi
    if (( pipeline_status[0] != 0 )); then
        mv -f -- "$working_report" "$failed_report"
        echo "Failed consistency report: $failed_report" >&2
        return "${pipeline_status[0]}"
    fi

    mv -f -- "$working_report" "$report"
    echo "Consistency report: $report"
}

command_export_media() {
    local run_dir="${1:-}"
    shift || true
    [[ -n "$run_dir" ]] || die "export-media requires a RUN_DIRECTORY"
    [[ -d "$run_dir" ]] || die "Run directory does not exist: $run_dir"
    run_dir="$(cd "$run_dir" && pwd)"
    local preset="$DEFAULT_PRESET"
    local output_dir="$run_dir/exported"
    while (( $# )); do
        case "$1" in
            --output) output_dir="${2:-}"; shift 2 ;;
            --preset) preset="${2:-}"; shift 2 ;;
            --help|-h) usage; return 0 ;;
            *) die "Unknown export-media option: $1" ;;
        esac
    done
    preset_python_version "$preset" >/dev/null
    [[ -n "$output_dir" ]] || die "--output requires a DIRECTORY"
    [[ "$output_dir" = /* ]] || output_dir="$PWD/$output_dir"
    local mcap="$run_dir/metadata.mcap"
    [[ -s "$mcap" ]] || die "Missing or empty MCAP: $mcap"
    [[ ! -e "$mcap.partial" ]] || die "Incomplete capture: found $mcap.partial"
    [[ ! -e "$output_dir" ]] || die "Export output already exists: $output_dir"
    local exporter
    exporter="$(resolve_exporter "$preset")" || return 1
    "$exporter" "$mcap" "$output_dir"
    echo "Export completed: $output_dir"
}

main() {
    local command="${1:-}"
    [[ -n "$command" ]] || { usage; return 1; }
    shift
    case "$command" in
        doctor) command_doctor "$@" ;;
        build) command_build "$@" ;;
        capabilities) command_capabilities "$@" ;;
        record) command_record "$@" ;;
        verify) command_verify "$@" ;;
        export-media) command_export_media "$@" ;;
        --help|-h|help) usage ;;
        *) die "Unknown command '$command'." ;;
    esac
}

main "$@"
