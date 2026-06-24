#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
OVERRIDES_DIR="${SCRIPT_DIR}/headless_overrides"
MANIFEST="${OVERRIDES_DIR}/manifest.txt"
BACKUP_ROOT="${SCRIPT_DIR}/backups"
PROFILE="headless"

ACTION="dry-run"
ASSUME_YES=0
BACKUP_DIR=""

usage() {
    cat <<'EOF'
Usage:
  tools/scripts/apply_headless_overrides.sh --dry-run [--profile headless|2026-full]
  tools/scripts/apply_headless_overrides.sh --apply [--profile headless|2026-full] [--yes]
  tools/scripts/apply_headless_overrides.sh --restore --backup-dir PATH [--yes]

Options:
  --dry-run          Show which repo files would be overwritten. This is the default.
  --apply            Copy headless override files into the AI Challenge repo.
  --restore          Restore files from a backup made by --apply.
  --profile NAME     Patch profile to use: headless (default) or 2026-full.
  --full-2026        Alias for --profile 2026-full.
  --backup-dir PATH  Backup directory to use for --restore.
  --repo-root PATH   Override the detected AI Challenge repo root.
  --yes, -y          Skip confirmation prompts.
  --help, -h         Show this help.
EOF
}

log() {
    echo "[headless-overrides] $*"
}

die() {
    echo "[headless-overrides][ERROR] $*" >&2
    exit 1
}

set_profile() {
    local profile="$1"
    [ -n "${profile}" ] || die "--profile requires NAME."
    case "${profile}" in
    headless)
        PROFILE="headless"
        MANIFEST="${OVERRIDES_DIR}/manifest.txt"
        ;;
    2026-full | full-2026)
        PROFILE="2026-full"
        MANIFEST="${OVERRIDES_DIR}/manifest.2026-full.txt"
        ;;
    *)
        die "Unknown profile: ${profile}. Expected: headless or 2026-full."
        ;;
    esac
}

confirm() {
    local prompt="$1"
    local answer=""

    if [ "${ASSUME_YES}" = "1" ]; then
        return 0
    fi
    if ! [ -r /dev/tty ]; then
        die "No TTY available for confirmation. Re-run with --yes if this is intentional."
    fi
    printf "[headless-overrides] %s [y/N]: " "${prompt}" >/dev/tty
    IFS= read -r answer </dev/tty || return 1
    case "${answer}" in
    y | Y | yes | YES)
        return 0
        ;;
    *)
        return 1
        ;;
    esac
}

iter_manifest() {
    [ -f "${MANIFEST}" ] || die "Manifest not found: ${MANIFEST}"
    while IFS= read -r rel || [ -n "${rel}" ]; do
        case "${rel}" in
        "" | \#*)
            continue
            ;;
        /* | *".."*)
            die "Refusing unsafe manifest path: ${rel}"
            ;;
        *)
            printf '%s\n' "${rel}"
            ;;
        esac
    done <"${MANIFEST}"
}

make_backup_dir() {
    if [ -n "${BACKUP_DIR}" ]; then
        printf '%s\n' "${BACKUP_DIR}"
        return 0
    fi
    printf '%s/%s-overrides-%s\n' "${BACKUP_ROOT}" "${PROFILE}" "$(date +%Y%m%d-%H%M%S)"
}

dry_run() {
    log "Repo root: ${REPO_ROOT}"
    log "Profile: ${PROFILE}"
    log "Override source: ${OVERRIDES_DIR}"
    log "Manifest: ${MANIFEST}"
    log "Files that would be overwritten:"
    iter_manifest | while IFS= read -r rel; do
        [ -f "${OVERRIDES_DIR}/${rel}" ] || die "Override file missing: ${OVERRIDES_DIR}/${rel}"
        printf '  %s -> %s\n' "${OVERRIDES_DIR}/${rel}" "${REPO_ROOT}/${rel}"
    done
}

apply_overrides() {
    local backup_dir
    backup_dir="$(make_backup_dir)"

    dry_run
    confirm "Apply ${PROFILE} override files and create backup at ${backup_dir}?" || die "Cancelled."

    mkdir -p "${backup_dir}"
    cp -p "${MANIFEST}" "${backup_dir}/manifest.txt"
    : >"${backup_dir}/missing-files.txt"

    iter_manifest | while IFS= read -r rel; do
        local src="${OVERRIDES_DIR}/${rel}"
        local dest="${REPO_ROOT}/${rel}"
        local backup="${backup_dir}/${rel}"

        [ -f "${src}" ] || die "Override file missing: ${src}"
        if [ -e "${dest}" ]; then
            mkdir -p "$(dirname -- "${backup}")"
            cp -p "${dest}" "${backup}"
        else
            printf '%s\n' "${rel}" >>"${backup_dir}/missing-files.txt"
        fi

        mkdir -p "$(dirname -- "${dest}")"
        cp -p "${src}" "${dest}"
        log "Applied: ${rel}"
    done

    log "Backup written: ${backup_dir}"
}

restore_overrides() {
    [ -n "${BACKUP_DIR}" ] || die "--restore requires --backup-dir PATH."
    [ -d "${BACKUP_DIR}" ] || die "Backup directory not found: ${BACKUP_DIR}"

    local restore_manifest="${BACKUP_DIR}/manifest.txt"
    [ -f "${restore_manifest}" ] || restore_manifest="${MANIFEST}"

    log "Repo root: ${REPO_ROOT}"
    log "Backup source: ${BACKUP_DIR}"
    confirm "Restore files from ${BACKUP_DIR}?" || die "Cancelled."

    while IFS= read -r rel || [ -n "${rel}" ]; do
        case "${rel}" in
        "" | \#*)
            continue
            ;;
        esac
        local src="${BACKUP_DIR}/${rel}"
        local dest="${REPO_ROOT}/${rel}"
        if [ -f "${src}" ]; then
            mkdir -p "$(dirname -- "${dest}")"
            cp -p "${src}" "${dest}"
            log "Restored: ${rel}"
        fi
    done <"${restore_manifest}"

    if [ -f "${BACKUP_DIR}/missing-files.txt" ]; then
        while IFS= read -r rel || [ -n "${rel}" ]; do
            [ -n "${rel}" ] || continue
            if [ -e "${REPO_ROOT}/${rel}" ]; then
                rm -f "${REPO_ROOT:?}/${rel}"
                log "Removed file that did not exist before apply: ${rel}"
            fi
        done <"${BACKUP_DIR}/missing-files.txt"
    fi
}

while [ $# -gt 0 ]; do
    case "$1" in
    --dry-run)
        ACTION="dry-run"
        shift
        ;;
    --apply)
        ACTION="apply"
        shift
        ;;
    --restore)
        ACTION="restore"
        shift
        ;;
    --profile)
        set_profile "${2-}"
        shift 2
        ;;
    --full-2026)
        set_profile "2026-full"
        shift
        ;;
    --backup-dir)
        BACKUP_DIR="${2-}"
        [ -n "${BACKUP_DIR}" ] || die "--backup-dir requires PATH."
        shift 2
        ;;
    --repo-root)
        REPO_ROOT="$(cd -- "${2-}" && pwd)"
        shift 2
        ;;
    --yes | -y)
        ASSUME_YES=1
        shift
        ;;
    --help | -h)
        usage
        exit 0
        ;;
    *)
        die "Unknown option: $1"
        ;;
    esac
done

case "${ACTION}" in
dry-run)
    dry_run
    ;;
apply)
    apply_overrides
    ;;
restore)
    restore_overrides
    ;;
*)
    die "Unknown action: ${ACTION}"
    ;;
esac
