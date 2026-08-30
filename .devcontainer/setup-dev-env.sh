#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo " Setting up Dev Environment & Zsh Tools"
echo "=========================================="

# 1. Install zsh, curl, fzf if missing
MISSING_PKGS=""
for pkg in zsh curl fzf; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
        MISSING_PKGS="$MISSING_PKGS $pkg"
    fi
done

if [ -n "$MISSING_PKGS" ]; then
    echo "[1/5] Installing missing packages:$MISSING_PKGS..."
    sudo apt-get update -y
    sudo apt-get install -y --no-install-recommends $MISSING_PKGS
    sudo rm -rf /var/lib/apt/lists/*
else
    echo "[1/5] Required packages (zsh, curl, fzf) already installed."
fi

# 2. Install Oh My Zsh
ZSH_DIR="$HOME/.oh-my-zsh"
if [ ! -d "$ZSH_DIR" ]; then
    echo "[2/5] Installing Oh My Zsh..."
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
else
    echo "[2/5] Oh My Zsh already installed."
fi

# 3. Install Zsh Plugins (Shallow clone)
CUSTOM_PLUGINS_DIR="${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins"
mkdir -p "$CUSTOM_PLUGINS_DIR"

if [ ! -d "$CUSTOM_PLUGINS_DIR/zsh-autosuggestions" ]; then
    echo "[3/5] Cloning zsh-autosuggestions..."
    git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions "$CUSTOM_PLUGINS_DIR/zsh-autosuggestions"
fi

if [ ! -d "$CUSTOM_PLUGINS_DIR/zsh-syntax-highlighting" ]; then
    echo "[3/5] Cloning zsh-syntax-highlighting..."
    git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git "$CUSTOM_PLUGINS_DIR/zsh-syntax-highlighting"
fi

if [ ! -d "$CUSTOM_PLUGINS_DIR/zsh-history-substring-search" ]; then
    echo "[3/5] Cloning zsh-history-substring-search..."
    git clone --depth=1 https://github.com/zsh-users/zsh-history-substring-search "$CUSTOM_PLUGINS_DIR/zsh-history-substring-search"
fi

# 4. Copy .zshrc configuration
echo "[4/5] Configuring ~/.zshrc..."
if [ -f "$SCRIPT_DIR/.zshrc" ]; then
    cp "$SCRIPT_DIR/.zshrc" "$HOME/.zshrc"
fi

# 5. Populate and persist Command History
echo "[5/5] Configuring persistent shell history..."
SEED_FILE="$SCRIPT_DIR/history_seed.txt"

# If /commandhistory volume exists
if [ -d "/commandhistory" ] && [ -w "/commandhistory" ]; then
    if [ ! -s "/commandhistory/.zsh_history" ] && [ -f "$SEED_FILE" ]; then
        cp "$SEED_FILE" "/commandhistory/.zsh_history"
    fi
    if [ ! -s "/commandhistory/.bash_history" ] && [ -f "$SEED_FILE" ]; then
        cp "$SEED_FILE" "/commandhistory/.bash_history"
    fi
fi

# Seed user home directory history if empty
if [ ! -s "$HOME/.zsh_history" ] && [ -f "$SEED_FILE" ]; then
    cp "$SEED_FILE" "$HOME/.zsh_history"
fi
if [ ! -s "$HOME/.bash_history" ] && [ -f "$SEED_FILE" ]; then
    cp "$SEED_FILE" "$HOME/.bash_history"
fi

# Set default user shell to zsh if possible
if [ "$SHELL" != "$(which zsh)" ] && [ -x "$(which zsh)" ]; then
    sudo chsh -s "$(which zsh)" "$USER" 2>/dev/null || true
fi

# Also configure ~/.bashrc fallback for standard ROS 2 environment
if ! grep -q "source /opt/ros" "$HOME/.bashrc" 2>/dev/null; then
    echo "source /opt/ros/\${ROS_DISTRO:-humble}/setup.bash" >> "$HOME/.bashrc"
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> "$HOME/.bashrc"
fi

# 6. Run rosdep update and install
echo "Updating rosdep dependencies..."
rosdep update || true
rosdep install --from-paths "$WORKSPACE_DIR/src" --ignore-src -r -y || true

# 7. Install agy
curl -fsSL https://antigravity.google/cli/install.sh | bash

echo "=========================================="
echo " Dev environment setup complete!"
echo "=========================================="
