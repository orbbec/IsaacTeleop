#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Use install.sh to fetch the SDK shared with EGO and prepare dependencies/udev.
# This helper maps build, discovery and capture commands to the mono Orbbec Wrist plugin.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
SOURCE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd -P)"

die() {
    echo "orbbec_wrist.sh: $*" >&2
    exit 1
}

usage() {
    cat <<'EOF'
Usage:
  orbbec_wrist.sh doctor [--sdk-root PATH] [--preset py3.11] [--plugin PATH]
  orbbec_wrist.sh build --sdk-root PATH [--preview] [--preset py3.11] [--jobs N] [--install-prefix PATH]
  orbbec_wrist.sh list [--sdk-root PATH] [--preset py3.11] [--plugin PATH]
  orbbec_wrist.sh record [--config FILE.toml] [--serial-number SN] [options]
  orbbec_wrist.sh verify RUN_DIRECTORY [--require-calibration]

Common options:
  --sdk-root PATH                  External SDK; defaults to ORBBEC_SDK_ROOT.
  --preset py3.11|py3.12|py3.13    Build Python version (default: py3.11).
  --plugin PATH                    Override the built/installed executable.

Record options (CLI overrides TOML regardless of --config position):
  --serial-number SN               Optional with one compatible Orbbec Wrist; required with multiple.
  --mcap-media-spool PATH          Additional embedded fragment; requires embedded mode and collection-prefix.
  --output DIRECTORY               New directory; default: recordings/wrist_<timestamp>.
  --duration SECONDS               Default: 0 (until Ctrl-C).
  --format h264|h265|mjpg          Default: h264.
  --fps N                         Default: 30; resolution is fixed at 1600x1200.
  --imu / --no-imu                 Default: enabled; no silent fallback.
  --preview / --no-preview         Show live Color (requires build --preview; default: off).
  --imu-rate 400|1000              Default: 400 Hz; device must advertise it.
  --accel-full-scale 16|24         Default: 24 g; gyro requests 2000 dps.
  --startup-timeout-ms N           Default: 5000.
  --stall-timeout-ms N             Default: 3000.
  --reconnect-timeout-ms N         Default: 30000; 0 disables recovery.
  --reconnect-interval-ms N        Default: 1000.
  --collection-prefix PREFIX      Publish live Color/Accel/Gyro metadata via OpenXR.
  --plugin-root-id ID              Optional device-status publisher identity.
  --media-mode sidecar|embedded    Default: sidecar (raw/Color.* plus metadata.mcap).
  --keep-media-sidecars / --no-keep-media-sidecars
                                  Retain raw/Color.* in embedded mode (default: off).
  --require-calibration           Verification requires Color intrinsics and raw YAML.
  --dry-run                       Print the command without creating output or opening hardware.
  -h, --help                      Show this help.

record writes raw/Color.*, metadata.mcap, calibration.json, capabilities.txt,
requested_profile.json and logs/capture.log, then verifies the recording.
Only successful verification creates verification_report.txt. Orbbec Wrist has no audio output.
Requested profiles and SDK/firmware compatibility require device validation.
Relative config paths are resolved against the calling working directory.
EOF
}

preset_version() {
    case "$1" in
        py3.11|py3.12|py3.13) printf '%s\n' "${1#py}" ;;
        *) die "Unknown preset '$1'; expected py3.11, py3.12 or py3.13." ;;
    esac
}

validate_sdk() {
    local root="$1" path
    [[ -n "$root" ]] || die "Use --sdk-root PATH or ORBBEC_SDK_ROOT to select an external SDK."
    for path in include/libobsensor/ObSensor.hpp lib/OrbbecSDKConfig.cmake lib/libOrbbecSDK.so.2; do
        [[ -f "$root/$path" ]] || die "Invalid SDK '$root': missing $path."
    done
    [[ -d "$root/lib/extensions" ]] || die "Invalid SDK '$root': missing lib/extensions."
}

require_cmake() {
    command -v cmake >/dev/null 2>&1 || die "Install CMake 3.24 or newer."
    local version
    version="$(cmake --version | awk 'NR == 1 {print $3}')"
    [[ "$(printf '%s\n%s\n' 3.24 "$version" | sort -V | head -n 1)" == 3.24 ]] \
        || die "CMake $version is too old; install CMake 3.24 or newer."
}

read_config() {
    local config="$1" candidate config_python=""
    [[ -f "$config" ]] || die "Configuration file not found: $config"
    for candidate in python3 python3.13 python3.12 python3.11; do
        if command -v "$candidate" >/dev/null 2>&1 \
                && "$candidate" -c 'import tomllib' >/dev/null 2>&1; then
            config_python="$candidate"
            break
        fi
    done
    [[ -n "$config_python" ]] || die "--config requires Python 3.11+ with tomllib."
    # Emit NUL-delimited arguments, never shell source, to preserve paths and serial numbers.
    "$config_python" - "$config" <<'PY'
import sys
import tomllib

strings = {"serial_number", "output", "format", "preset", "plugin", "sdk_root",
           "collection_prefix", "plugin_root_id", "media_mode", "mcap_media_spool"}
numbers = {"duration", "fps", "imu_rate", "accel_full_scale",
           "startup_timeout_ms", "stall_timeout_ms", "reconnect_timeout_ms",
           "reconnect_interval_ms"}
booleans = {"imu", "keep_media_sidecars", "preview", "require_calibration"}
try:
    with open(sys.argv[1], "rb") as source:
        document = tomllib.load(source)
    if set(document) != {"record"} or not isinstance(document["record"], dict):
        raise ValueError("configuration must contain only a [record] table")
    arguments = []
    for key, value in document["record"].items():
        option = "--" + key.replace("_", "-")
        if key in strings and type(value) is str and value and "\0" not in value:
            arguments.extend((option, value))
        elif key in numbers and type(value) is int and 0 <= value <= 4294967295:
            arguments.extend((option, str(value)))
        elif key in booleans and type(value) is bool:
            arguments.append(option if value else "--no-" + key.replace("_", "-"))
        else:
            raise ValueError(f"unknown option or invalid value for record.{key}")
    for argument in arguments:
        sys.stdout.buffer.write(argument.encode("utf-8") + b"\0")
except (OSError, ValueError) as error:
    sys.exit(f"orbbec_wrist.sh: {error}")
PY
}

command="${1:---help}"
shift || true
case "$command" in
    --help|-h|help) usage; exit 0 ;;
    doctor|build|list|record|verify) ;;
    *) die "Unknown command '$command'; run with --help." ;;
esac

if [[ "$command" == verify ]]; then
    exec python3 "$SCRIPT_DIR/tools/verify_recording.py" "$@"
fi

declare -A settings=(
    [sdk-root]="${ORBBEC_SDK_ROOT:-}" [preset]=py3.11 [plugin]=""
    [jobs]="" [install-prefix]="" [serial-number]="" [output]=""
    [duration]=0 [format]=h264 [fps]=30 [media-mode]=sidecar
    [imu-rate]=400 [accel-full-scale]=24 [startup-timeout-ms]=5000
    [stall-timeout-ms]=3000 [reconnect-timeout-ms]=30000 [reconnect-interval-ms]=1000
    [collection-prefix]="" [plugin-root-id]="" [mcap-media-spool]=""
)
imu=1
sidecars=0
preview=0
dry_run=0
require_calibration=0
original_args=("$@")
config=""
if [[ "$command" == record ]]; then
    for (( index=0; index<${#original_args[@]}; ++index )); do
        case "${original_args[index]}" in
            --help|-h) usage; exit 0 ;;
            --config)
                (( index+1<${#original_args[@]} )) || die "--config requires a file."
                [[ -z "$config" ]] || die "Specify --config only once."
                config="${original_args[index+1]}"
                [[ -n "$config" && "$config" != --* ]] || die "--config requires a file."
                (( ++index ))
                ;;
        esac
    done
fi
config_args=()
if [[ -n "$config" ]]; then
    config_args_file="$(mktemp)"
    if ! read_config "$config" > "$config_args_file"; then
        rm -f -- "$config_args_file"
        exit 1
    fi
    mapfile -d '' -t config_args < "$config_args_file"
    rm -f -- "$config_args_file"
fi
set -- "${config_args[@]}" "${original_args[@]}"
while (( $# )); do
    case "$command:$1" in
        *:--help|*:-h) usage; exit 0 ;;
        record:--imu) imu=1; shift; continue ;;
        record:--no-imu) imu=0; shift; continue ;;
        build:--preview|record:--preview) preview=1; shift; continue ;;
        build:--no-preview|record:--no-preview) preview=0; shift; continue ;;
        record:--keep-media-sidecars) sidecars=1; shift; continue ;;
        record:--no-keep-media-sidecars) sidecars=0; shift; continue ;;
        record:--require-calibration) require_calibration=1; shift; continue ;;
        record:--no-require-calibration) require_calibration=0; shift; continue ;;
        record:--dry-run) dry_run=1; shift; continue ;;
        record:--config) shift 2; continue ;;
        *:--sdk-root|*:--preset|doctor:--plugin|list:--plugin|record:--plugin|\
        build:--jobs|build:--install-prefix|\
        record:--serial-number|record:--output|record:--duration|record:--format|\
        record:--fps|record:--media-mode|record:--imu-rate|\
        record:--accel-full-scale|record:--startup-timeout-ms|record:--stall-timeout-ms|\
        record:--reconnect-timeout-ms|record:--reconnect-interval-ms|\
        record:--collection-prefix|record:--plugin-root-id|record:--mcap-media-spool)
            [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 requires a value."
            settings["${1#--}"]="$2"
            shift 2
            ;;
        *) die "Unknown $command option '$1'; run with --help." ;;
    esac
done

version="$(preset_version "${settings[preset]}")"
build_dir="$SOURCE_ROOT/build-wrist-py$version"
sdk_root="${settings[sdk-root]}"
if [[ -n "$sdk_root" ]]; then
    validate_sdk "$sdk_root"
    sdk_root="$(cd "$sdk_root" && pwd -P)"
    export LD_LIBRARY_PATH="$sdk_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

resolve_plugin() {
    local plugin="${settings[plugin]}"
    if [[ -z "$plugin" ]]; then
        if [[ -x "$SCRIPT_DIR/orbbec_wrist_camera_plugin" ]]; then
            plugin="$SCRIPT_DIR/orbbec_wrist_camera_plugin"
        else
            plugin="$build_dir/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin"
        fi
    fi
    [[ -x "$plugin" && -f "$plugin" ]] || die "Plugin is not executable: $plugin. Run build or pass --plugin PATH."
    printf '%s/%s\n' "$(cd "$(dirname "$plugin")" && pwd -P)" "$(basename "$plugin")"
}

case "$command" in
    build)
        [[ -f "$SOURCE_ROOT/src/plugins/orbbec_wrist/CMakeLists.txt" ]] \
            || die "Build requires an Isaac Capture source checkout."
        validate_sdk "$sdk_root"
        [[ -z "${settings[jobs]}" || "${settings[jobs]}" =~ ^[1-9][0-9]*$ ]] \
            || die "--jobs must be a positive integer."
        require_cmake
        command -v uv >/dev/null 2>&1 || die "Install uv before building Isaac Capture."
        command -v c++ >/dev/null 2>&1 || die "Install a C++ compiler (build-essential)."
        cmake -S "$SOURCE_ROOT" -B "$build_dir" \
            -DISAAC_TELEOP_PYTHON_VERSION="$version" -DBUILD_VIZ=OFF \
            -DORBBEC_WRIST_ENABLE_PREVIEW="$preview" \
            -DBUILD_PLUGINS=ON -DBUILD_PLUGIN_ORBBEC_WRIST_CAMERA=ON -DORBBEC_SDK_ROOT="$sdk_root"
        build_args=(--build "$build_dir" --target orbbec_wrist_camera_plugin --parallel)
        [[ -z "${settings[jobs]}" ]] || build_args+=("${settings[jobs]}")
        cmake "${build_args[@]}"
        if [[ -n "${settings[install-prefix]}" ]]; then
            cmake --install "$build_dir" --prefix "${settings[install-prefix]}" --component orbbec_wrist
        fi
        echo "Plugin: $build_dir/src/plugins/orbbec_wrist/orbbec_wrist_camera_plugin"
        ;;
    doctor)
        [[ "$(uname -s)" == Linux ]] || die "The current Orbbec Wrist backend requires Linux."
        echo "Host: $(uname -s) $(uname -m)"
        echo "Hardware-validated SDK/firmware combinations: not yet established."
        if [[ -f "$SOURCE_ROOT/src/plugins/orbbec_wrist/CMakeLists.txt" ]]; then
            require_cmake
            command -v uv >/dev/null 2>&1 || die "Install uv before building Isaac Capture."
            command -v c++ >/dev/null 2>&1 || die "Install a C++ compiler."
            validate_sdk "$sdk_root"
        fi
        if [[ -n "$sdk_root" ]]; then
            echo "External SDK: $sdk_root"
            if [[ -f "$sdk_root/lib/OrbbecSDKVersion.cmake" ]]; then
                awk '/^set\(PACKAGE_VERSION / {print "SDK " $0; exit}' "$sdk_root/lib/OrbbecSDKVersion.cmake"
            fi
        fi
        if plugin="$(resolve_plugin)"; then
            "$plugin" --help >/dev/null || die "Plugin cannot start; check external SDK/runtime libraries."
            echo "Plugin loader check: OK ($plugin)"
        else
            die "Build the plugin before running doctor."
        fi
        if command -v lsusb >/dev/null 2>&1; then
            usb_devices="$(lsusb -d 2bc5:1203 || true)"
            if [[ -n "$usb_devices" ]]; then
                printf '%s\n' "$usb_devices"
                while read -r _ bus _ device _; do
                    node="/dev/bus/usb/$bus/${device%:}"
                    [[ -r "$node" && -w "$node" ]] || die "USB access denied: $node. Install SDK udev rules and reconnect."
                done <<< "$usb_devices"
            else
                echo "No Orbbec Wrist USB device (2bc5:1203) found; hardware checks remain pending."
            fi
        else
            echo "Install usbutils for USB discovery/access checks."
        fi
        echo "Doctor checked setup only; run list and a real capture for device validation."
        ;;
    list)
        plugin="$(resolve_plugin)"
        exec "$plugin" --list-devices
        ;;
    record)
        case "${settings[format]}" in
            h264|h265|mjpg) ;;
            *) die "--format must be h264, h265 or mjpg." ;;
        esac
        for key in duration fps imu-rate accel-full-scale startup-timeout-ms \
                stall-timeout-ms reconnect-timeout-ms reconnect-interval-ms; do
            value="${settings[$key]}"
            [[ "$value" =~ ^(0|[1-9][0-9]*)$ && ${#value} -le 10 ]] \
                || die "--$key must be an unsigned integer."
            (( value <= 4294967295 )) || die "--$key exceeds uint32."
            case "$key" in
                fps) (( value <= 2147483647 )) || die "--$key exceeds int32." ;;
                startup-timeout-ms|stall-timeout-ms|reconnect-interval-ms)
                    (( value > 0 )) || die "--$key must be positive." ;;
            esac
        done
        [[ "${settings[imu-rate]}" == 400 || "${settings[imu-rate]}" == 1000 ]] || die "--imu-rate must be 400 or 1000."
        [[ "${settings[accel-full-scale]}" == 16 || "${settings[accel-full-scale]}" == 24 ]] || die "--accel-full-scale must be 16 or 24."
        [[ "${settings[media-mode]}" == sidecar || "${settings[media-mode]}" == embedded ]] \
            || die "--media-mode must be sidecar or embedded."
        if [[ -n "${settings[mcap-media-spool]}" ]]; then
            [[ "${settings[media-mode]}" == embedded && -n "${settings[collection-prefix]}" ]] \
                || die "--mcap-media-spool requires --media-mode embedded and --collection-prefix."
        fi
        plugin="$(resolve_plugin)"
        output_base="$PWD"
        [[ ! -f "$SOURCE_ROOT/src/plugins/orbbec_wrist/CMakeLists.txt" ]] || output_base="$SOURCE_ROOT"
        run_dir="${settings[output]:-$output_base/recordings/wrist_$(date +%Y%m%d_%H%M%S)}"
        [[ "$run_dir" == /* ]] || run_dir="$PWD/$run_dir"
        [[ ! -e "$run_dir" && ! -L "$run_dir" ]] || die "Output path already exists: $run_dir"
        plugin_args=("--mcap-filename=$run_dir/metadata.mcap" "--media-mode=${settings[media-mode]}"
                     "--calibration-output=$run_dir/calibration.json" "--capabilities-output=$run_dir/capabilities.txt")
        [[ -z "${settings[serial-number]}" ]] || plugin_args+=("--serial-number=${settings[serial-number]}")
        for key in duration format fps imu-rate accel-full-scale startup-timeout-ms \
                stall-timeout-ms reconnect-timeout-ms reconnect-interval-ms collection-prefix plugin-root-id mcap-media-spool; do
            [[ -z "${settings[$key]}" ]] || plugin_args+=("--$key=${settings[$key]}")
        done
        (( imu )) || plugin_args+=(--disable-imu)
        (( !preview )) || plugin_args+=(--preview)
        if [[ "${settings[media-mode]}" == sidecar ]] || (( sidecars )); then
            plugin_args+=("--video-output=$run_dir/raw/Color.${settings[format]}")
        fi
        if (( dry_run )); then
            if [[ -n "$sdk_root" ]]; then
                printf '%q ' env "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
            fi
            printf '%q ' "$plugin" "${plugin_args[@]}"
            printf '\n'
            exit 0
        fi
        command -v python3 >/dev/null || die "Recording verification requires Python 3."
        command -v ffmpeg >/dev/null || die "Recording verification requires ffmpeg."
        command -v ffprobe >/dev/null || die "Recording verification requires ffprobe."
        python3 -c 'from mcap.reader import make_reader' 2>/dev/null \
            || die "Install the Python MCAP reader: python3 -m pip install --user mcap"
        mkdir -p -- "$(dirname "$run_dir")"
        mkdir -- "$run_dir"
        mkdir -- "$run_dir/logs"
        python3 - "$run_dir/requested_profile.json" "${settings[format]}" "${settings[fps]}" \
            "${settings[imu-rate]}" "$imu" "${settings[media-mode]}" "${settings[serial-number]}" <<'PYMANIFEST'
import json
import sys
from pathlib import Path
path, codec, fps, rate, imu, mode, serial = sys.argv[1:]
manifest = {"version": 1, "streams": {"Color": {"format": codec, "width": 1600,
            "height": 1200, "fps": int(fps)}}, "imu": {"enabled": imu == "1",
            "sample_rate_hz": int(rate)}, "media_mode": mode, "serial_number": serial}
Path(path).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
PYMANIFEST
        echo "Recording: $run_dir (1600x1200 Color; ${settings[media-mode]} media)."
        echo "Stop with Ctrl-C once; capture is verified after finalization."
        # Keep tee alive for the plugin's final shutdown output after terminal SIGINT.
        trap ':' INT
        set +e
        "$plugin" "${plugin_args[@]}" 2>&1 | tee -i "$run_dir/logs/capture.log"
        capture_statuses=("${PIPESTATUS[@]}")
        set -e
        trap - INT
        (( capture_statuses[1] == 0 )) || die "Cannot write capture log: $run_dir/logs/capture.log"
        (( capture_statuses[0] == 0 )) || die "Capture failed; inspect $run_dir/logs/capture.log"
        verify_args=("$run_dir")
        (( !require_calibration )) || verify_args+=(--require-calibration)
        python3 "$SCRIPT_DIR/tools/verify_recording.py" "${verify_args[@]}"
        ;;
esac
