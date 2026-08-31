export USER="${USER:-$(id -un)}"

# ------------------------------------------------------------------------------
# 1. ble.sh (Bash Line Editor) - Attach (Part 1: before prompt)
# ------------------------------------------------------------------------------
if [[ $- == *i* ]] && [ -f "$HOME/.local/share/blesh/ble.sh" ]; then
    source "$HOME/.local/share/blesh/ble.sh" --noattach
fi

# ------------------------------------------------------------------------------
# 2. Oh My Bash Configuration
# ------------------------------------------------------------------------------
export OSH="$HOME/.oh-my-bash"
OSH_THEME="robbyrussell"

# Plugins (git status, sudo shortcut, colored man pages, etc.)
plugins=(
    git
    sudo
    colored-man-pages
)

# Load Oh My Bash
if [ -d "$OSH" ] && [ -f "$OSH/oh-my-bash.sh" ]; then
    source "$OSH/oh-my-bash.sh"
fi

# ------------------------------------------------------------------------------
# 3. Persistent Shell History
# ------------------------------------------------------------------------------
if [ -d "/commandhistory" ] && [ -w "/commandhistory" ]; then
    export HISTFILE="/commandhistory/.bash_history"
else
    export HISTFILE="$HOME/.bash_history"
fi
export HISTSIZE=50000
export HISTFILESIZE=50000
export HISTCONTROL=ignoreboth:erasedups
shopt -s histappend

# ------------------------------------------------------------------------------
# 4. ROS 2 & Colcon Environment 
# ------------------------------------------------------------------------------
# Source ROS Underlay
if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

# Colcon Command-line Tab Autocomplete
if [ -f "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
    source "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
fi

# colcon_cd helper (Quick jump to package directory)
if [ -f "/usr/share/colcon_cd/function/colcon_cd.sh" ]; then
    source "/usr/share/colcon_cd/function/colcon_cd.sh"
    export _colcon_cd_root="${PWD}"
fi

# Auto-source local workspace overlay if present
if [ -f "install/setup.bash" ]; then
    source "install/setup.bash"
elif [ -f "/workspaces/ros-cdt/install/setup.bash" ]; then
    source "/workspaces/ros-cdt/install/setup.bash"
fi

# User local bin path
if [ -d "$HOME/.local/bin" ]; then
    export PATH="$HOME/.local/bin:$PATH"
fi

if [[ ${BLE_VERSION-} ]]; then
    ble-attach
fi
