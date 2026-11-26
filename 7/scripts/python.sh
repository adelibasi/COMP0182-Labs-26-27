#!/usr/bin/env bash
# process-local isolation: apt OpenCV/NumPy and Humble, never pip/site packages
set -eo pipefail
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
exec /usr/bin/python3 -I -S - "$@" <<'PY'
import runpy
import sys
from pathlib import Path

sys.path[:0] = [
    "/usr/lib/python3/dist-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
]
args = sys.argv[1:]
if not args:
    raise SystemExit("Usage: bash scripts/python.sh SCRIPT [args], or -m MODULE [args]")
if args[0] == "-m":
    if len(args) < 2:
        raise SystemExit("A module name is required after -m")
    sys.argv = args[1:]
    runpy.run_module(args[1], run_name="__main__", alter_sys=True)
else:
    path = Path(args[0]).resolve()
    sys.path.insert(0, str(path.parent))
    sys.argv = args
    runpy.run_path(str(path), run_name="__main__")
PY
