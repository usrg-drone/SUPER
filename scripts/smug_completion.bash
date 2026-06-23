# bash completion for smug (tmux session manager)
# Source this file or install to /etc/bash_completion.d/smug

_smug_projects() {
    # Find smug binary: prefer system PATH, fall back to vendored copy.
    local _smug
    if command -v smug >/dev/null 2>&1; then
        _smug=smug
    elif [[ -x "${SUPER_WS:-${HOME}/super_ws}/src/scripts/bin/smug" ]]; then
        _smug="${SUPER_WS:-${HOME}/super_ws}/src/scripts/bin/smug"
    else
        return
    fi
    "${_smug}" list 2>/dev/null
}

_smug() {
    local cur prev words cword
    _init_completion 2>/dev/null || {
        COMPREPLY=()
        cur="${COMP_WORDS[COMP_CWORD]}"
        prev="${COMP_WORDS[COMP_CWORD-1]}"
        words=("${COMP_WORDS[@]}")
        cword=$COMP_CWORD
    }

    local commands="list edit new start stop print rm switch"

    if [[ $cword -eq 1 ]]; then
        COMPREPLY=($(compgen -W "$commands" -- "$cur"))
        return
    fi

    case "${words[1]}" in
        start|stop|edit|print|rm|switch)
            if [[ $cword -eq 2 ]]; then
                local projects
                projects=$(_smug_projects)
                COMPREPLY=($(compgen -W "$projects" -- "$cur"))
            fi
            ;;
    esac
}

complete -F _smug smug
