#!/usr/bin/env bash
# Downloads a Ros2ForUnity release and extracts it to Assets/Ros2ForUnity.
# Run from the repository root: ./Util/Ros2ForUnity/DownloadRos2ForUnity.sh [tag]
# If no tag is given, the latest release is used.
# Requires: gh (GitHub CLI), unzip
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO="ub-cavas/UB-MR"
TMP_DIR="$(mktemp -d)"

echo "Downloading Ros2ForUnity release..."
gh release download ${1:+"$1"} \
  --repo "$REPO" \
  --pattern "Ros2ForUnity.zip" \
  --dir "$TMP_DIR"

echo "Extracting to Assets/Ros2ForUnity..."
rm -rf "${REPO_ROOT}/Assets/Ros2ForUnity"
rm -f "${REPO_ROOT}/Assets/Ros2ForUnity.meta"
unzip -q "${TMP_DIR}/Ros2ForUnity.zip" -d "${REPO_ROOT}/Assets/"
rm -rf "$TMP_DIR"

echo "Done."
