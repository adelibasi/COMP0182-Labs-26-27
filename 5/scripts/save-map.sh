#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    printf 'Usage: %s <relative-path.yaml>\n' "$0" >&2
    exit 2
fi

map_file=$1
case "$map_file" in
    ""|/*|../*|*/../*|*/..|.yaml|*/.yaml)
        printf 'Use a named, relative .yaml path without .. components.\n' >&2
        exit 2
        ;;
esac
case "$map_file" in
    *.yaml) ;;
    *) printf 'The output path must end in .yaml.\n' >&2; exit 2 ;;
esac

use_sim_time=${USE_SIM_TIME-true}
case "$use_sim_time" in
    true|false) ;;
    *) printf 'USE_SIM_TIME must be true or false.\n' >&2; exit 2 ;;
esac

map_base=${map_file%.yaml}
# shortcut: single saver per path; use atomic exports for concurrent saving
for output in "$map_file" "$map_base.pgm"; do
    if [[ -e "$output" || -L "$output" ]]; then
        printf 'Refusing to overwrite %s\n' "$output" >&2
        exit 2
    fi
done

mkdir -p -- "$(dirname -- "$map_file")"
exec ros2 run nav2_map_server map_saver_cli -t /map -f "$map_base" --fmt pgm --mode trinary \
    --ros-args -p "use_sim_time:=$use_sim_time" -p save_map_timeout:=10.0 \
    -p map_subscribe_transient_local:=true
