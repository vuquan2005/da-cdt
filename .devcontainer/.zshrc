# Path to oh-my-zsh installation
export ZSH="$HOME/.oh-my-zsh"

# Theme
ZSH_THEME="robbyrussell"

# Plugins
plugins=(
    git
    zsh-autosuggestions
    zsh-history-substring-search
    zsh-syntax-highlighting
)

# Load Oh My Zsh
if [ -d "$ZSH" ]; then
    source "$ZSH/oh-my-zsh.sh"
fi

# ------------------------------------------------------------------------------
# Persistent Shell History
# ------------------------------------------------------------------------------
if [ -d "/commandhistory" ] && [ -w "/commandhistory" ]; then
    export HISTFILE="/commandhistory/.zsh_history"
else
    export HISTFILE="$HOME/.zsh_history"
fi
export HISTSIZE=50000
export SAVEHIST=50000
setopt APPEND_HISTORY
setopt SHARE_HISTORY
setopt HIST_IGNORE_ALL_DUPS
setopt HIST_SAVE_NO_DUPS
setopt HIST_FIND_NO_DUPS
setopt HIST_REDUCE_BLANKS

# ------------------------------------------------------------------------------
# Keybindings for zsh-history-substring-search
# (Press Up/Down arrow to search commands matching what you have typed)
# ------------------------------------------------------------------------------
bindkey '^[[A' history-substring-search-up
bindkey '^[[B' history-substring-search-down
[[ -n "${terminfo[kcuu1]}" ]] && bindkey "${terminfo[kcuu1]}" history-substring-search-up
[[ -n "${terminfo[kcud1]}" ]] && bindkey "${terminfo[kcud1]}" history-substring-search-down

# ------------------------------------------------------------------------------
# ROS 2 & Colcon Environment 
# ------------------------------------------------------------------------------
# 1. Source ROS Underlay
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.zsh"
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
fi

# 2. Colcon Command-line Tab Autocomplete
if [ -f "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh" ]; then
    source "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh"
fi

# 3. colcon_cd helper (Quick jump to package directory)
if [ -f "/usr/share/colcon_cd/function/colcon_cd.sh" ]; then
    source "/usr/share/colcon_cd/function/colcon_cd.sh"
    export _colcon_cd_root="${PWD}"
fi

# 4. Auto-source local workspace overlay if present
if [ -f "install/setup.zsh" ]; then
    source "install/setup.zsh"
elif [ -f "/workspaces/ros-cdt/install/setup.zsh" ]; then
    source "/workspaces/ros-cdt/install/setup.zsh"
fi

# User local bin path
if [ -d "$HOME/.local/bin" ]; then
    export PATH="$HOME/.local/bin:$PATH"
fi
