#!/bin/bash
set -e

REPO="ub-cavas/UB-MR"
VERSION="${1:-latest}"
BUILDS_DIR="$(dirname "$0")/Builds"

# Resolve the release API URL
if [ "$VERSION" = "latest" ]; then
    API_URL="https://api.github.com/repos/${REPO}/releases/latest"
else
    API_URL="https://api.github.com/repos/${REPO}/releases/tags/${VERSION}"
fi

echo "=== Unity Player Installer ==="
echo "Fetching release info for: ${VERSION}"

# Fetch release metadata
RELEASE_JSON=$(curl -fsSL "$API_URL")

TAG=$(echo "$RELEASE_JSON" | grep -m1 '"tag_name"' | sed 's/.*"tag_name": *"\([^"]*\)".*/\1/')
echo "Found release: ${TAG}"

# Get the first asset download URL (prefers .zip or .tar.gz build archives)
ASSET_URL=$(echo "$RELEASE_JSON" | grep '"browser_download_url"' | grep -E '\.(zip|tar\.gz)' | head -1 | sed 's/.*"browser_download_url": *"\([^"]*\)".*/\1/')

if [ -z "$ASSET_URL" ]; then
    # Fall back to the first asset if no zip/tar.gz found
    ASSET_URL=$(echo "$RELEASE_JSON" | grep '"browser_download_url"' | head -1 | sed 's/.*"browser_download_url": *"\([^"]*\)".*/\1/')
fi

if [ -z "$ASSET_URL" ]; then
    echo "ERROR: No downloadable assets found for release ${TAG}."
    echo "Please download manually: https://github.com/${REPO}/releases/tag/${TAG}"
    exit 1
fi

FILENAME=$(basename "$ASSET_URL")
echo "Downloading: ${FILENAME}"

# Create Builds directory if needed
mkdir -p "$BUILDS_DIR"

TMP_FILE=$(mktemp)
curl -fL --progress-bar -o "$TMP_FILE" "$ASSET_URL"

echo "Extracting into Builds/..."
case "$FILENAME" in
    *.zip)
        unzip -q "$TMP_FILE" -d "$BUILDS_DIR"
        ;;
    *.tar.gz | *.tgz)
        tar -xzf "$TMP_FILE" -C "$BUILDS_DIR"
        ;;
    *)
        echo "WARNING: Unknown archive format '${FILENAME}'. Copying as-is to Builds/."
        cp "$TMP_FILE" "${BUILDS_DIR}/${FILENAME}"
        ;;
esac

rm -f "$TMP_FILE"

echo "Done. Unity Player installed to: ${BUILDS_DIR}"
