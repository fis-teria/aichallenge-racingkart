#!/usr/bin/env bash
set -euo pipefail

output_root="${ROSBAG_CLEAN_ROOT:-/output}"
retention_minutes="${ROSBAG_RETENTION_MINUTES:-180}"
active_grace_minutes="${ROSBAG_ACTIVE_GRACE_MINUTES:-10}"
interval_seconds="${ROSBAG_CLEAN_INTERVAL_SECONDS:-300}"
max_total_gb="${ROSBAG_MAX_TOTAL_GB:-20}"
run_once=false

if [[ "${1:-}" == "--once" ]]; then
    run_once=true
elif [[ -n "${1:-}" ]]; then
    echo "usage: $0 [--once]" >&2
    exit 2
fi

if [[ "$output_root" != /* || "$output_root" == "/" ]]; then
    echo "refusing unsafe ROSBAG_CLEAN_ROOT: $output_root" >&2
    exit 2
fi
for value in "$retention_minutes" "$active_grace_minutes" "$interval_seconds" "$max_total_gb"; do
    if [[ ! "$value" =~ ^[0-9]+$ ]]; then
        echo "cleanup settings must be non-negative integers" >&2
        exit 2
    fi
done

delete_bag_directory() {
    local directory="$1"
    local resolved_root resolved_directory
    resolved_root="$(realpath -m "$output_root")"
    resolved_directory="$(realpath -m "$directory")"
    if [[ "$resolved_directory" != "$resolved_root"/* ||
          "$(basename "$resolved_directory")" != rosbag2* ]]; then
        echo "refusing unsafe cleanup target: $directory" >&2
        return 1
    fi
    echo "[rosbag-cleaner] deleting $resolved_directory"
    find "$resolved_directory" -depth -delete
}

is_active() {
    local directory="$1"
    find "$directory" -type f -mmin "-${active_grace_minutes}" -print -quit |
        grep -q .
}

cleanup_once() {
    [[ -d "$output_root" ]] || return 0

    mapfile -d '' bag_directories < <(
        find "$output_root" -xdev -type d -name 'rosbag2*' -print0
    )

    local directory
    for directory in "${bag_directories[@]}"; do
        if is_active "$directory"; then
            continue
        fi
        if ! find "$directory" -type f -mmin "-${retention_minutes}" -print -quit |
            grep -q .
        then
            delete_bag_directory "$directory"
        fi
    done

    local maximum_bytes=$((max_total_gb * 1024 * 1024 * 1024))
    (( maximum_bytes > 0 )) || return 0
    mapfile -t oldest_first < <(
        find "$output_root" -xdev -type d -name 'rosbag2*' -printf '%T@ %p\n' |
            sort -n | cut -d' ' -f2-
    )
    local total_bytes=0
    for directory in "${oldest_first[@]}"; do
        [[ -d "$directory" ]] || continue
        total_bytes=$((total_bytes + $(du -sb "$directory" | cut -f1)))
    done
    for directory in "${oldest_first[@]}"; do
        (( total_bytes <= maximum_bytes )) && break
        [[ -d "$directory" ]] || continue
        is_active "$directory" && continue
        local directory_bytes
        directory_bytes="$(du -sb "$directory" | cut -f1)"
        delete_bag_directory "$directory"
        total_bytes=$((total_bytes - directory_bytes))
    done
}

while true; do
    cleanup_once
    "$run_once" && break
    sleep "$interval_seconds"
done
