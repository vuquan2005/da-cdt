#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo " Setting up Dev Environment (Oh My Bash & ble.sh)"
echo "=========================================="

# 1. Install Oh My Bash (Shallow clone)
OSH_DIR="$HOME/.oh-my-bash"
if [ ! -d "$OSH_DIR" ]; then
    echo "[1/4] Installing Oh My Bash..."
    git clone --depth=1 https://github.com/ohmybash/oh-my-bash.git "$OSH_DIR"
else
    echo "[1/4] Oh My Bash already installed."
fi

# 2. Install ble.sh (Pre-built binary for real-time syntax highlighting & autosuggestions)
BLE_DIR="$HOME/.local/share/blesh"
if [ ! -f "$BLE_DIR/ble.sh" ]; then
    echo "[2/4] Installing ble.sh..."
    mkdir -p "$HOME/.local/share"
    curl -fsSL https://github.com/akinomyoga/ble.sh/releases/download/nightly/ble-nightly.tar.xz | tar xJf - -C "$HOME/.local/share/"
    mv "$HOME/.local/share/ble-nightly" "$BLE_DIR"
else
    echo "[2/4] ble.sh already installed."
fi

# 3. Configure ~/.bashrc
echo "[3/4] Configuring ~/.bashrc..."
if [ -f "$SCRIPT_DIR/.bashrc" ]; then
    cp "$SCRIPT_DIR/.bashrc" "$HOME/.bashrc"
fi

# 4. Populate and persist Command History
echo "[4/4] Configuring persistent shell history..."
SEED_FILE="$SCRIPT_DIR/history_seed.txt"

if [ -d "/commandhistory" ] && [ -w "/commandhistory" ]; then
    if [ ! -s "/commandhistory/.bash_history" ] && [ -f "$SEED_FILE" ]; then
        cp "$SEED_FILE" "/commandhistory/.bash_history"
    fi
fi

if [ ! -s "$HOME/.bash_history" ] && [ -f "$SEED_FILE" ]; then
    cp "$SEED_FILE" "$HOME/.bash_history"
fi

# 5. Install agy CLI if needed
if ! command -v agy >/dev/null 2>&1; then
    curl -fsSL https://antigravity.google/cli/install.sh | bash || true
fi

echo "=========================================="
echo " Dev environment setup complete!"
echo "=========================================="
