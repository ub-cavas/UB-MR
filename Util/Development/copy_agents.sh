#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}/Agents"

COMPANY_NAME="${UNITY_COMPANY_NAME:-UB-CAVAS}"
PRODUCT_NAME="${UNITY_PRODUCT_NAME:-UB-MR}"

if [[ ! -d "${SOURCE_DIR}" ]]; then
  echo "Error: source directory '${SOURCE_DIR}' does not exist."
  exit 1
fi

case "$(uname -s)" in
  Linux)
    DEFAULT_DEST_DIR="${HOME}/.config/unity3d/${COMPANY_NAME}/${PRODUCT_NAME}"
    ;;
  Darwin)
    DEFAULT_DEST_DIR="${HOME}/Library/Application Support/${COMPANY_NAME}/${PRODUCT_NAME}"
    ;;
  CYGWIN*|MINGW*|MSYS*)
    APPDATA_DIR="${APPDATA:-}"
    if [[ -z "${APPDATA_DIR}" ]]; then
      echo "Error: APPDATA is not set, so the Windows Unity persistent data path could not be resolved."
      exit 1
    fi
    DEFAULT_DEST_DIR="${APPDATA_DIR}/../LocalLow/${COMPANY_NAME}/${PRODUCT_NAME}"
    ;;
  *)
    echo "Error: unsupported platform '$(uname -s)'."
    exit 1
    ;;
esac

DEST_DIR="${1:-${UNITY_PERSISTENT_DATA_PATH:-${DEFAULT_DEST_DIR}}}"

mkdir -p "${DEST_DIR}"

shopt -s nullglob
json_files=("${SOURCE_DIR}"/*.json)
shopt -u nullglob

if [[ ${#json_files[@]} -eq 0 ]]; then
  echo "No JSON files found in '${SOURCE_DIR}'."
  exit 0
fi

cp "${json_files[@]}" "${DEST_DIR}/"

echo "Copied ${#json_files[@]} agent file(s) to '${DEST_DIR}'."
