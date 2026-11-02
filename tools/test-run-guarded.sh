#!/bin/bash
# Pins run-guarded.py's promise: nothing a job starts survives it, including a grandchild in its
# own session (how Playwright starts Chromium), and the job's own SIGTERM handler gets to run.
set -u
cd "$(dirname "$0")/.."
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
fail=0

# A job that starts a detached grandchild, then waits. On SIGTERM it records that cleanup ran.
cat > "$T/job.py" <<'E'
import os, signal, subprocess, sys, time
tag, marker = sys.argv[1], sys.argv[2]
subprocess.Popen([sys.executable, "-c", f"import time; time.sleep(300) # {tag}-grandchild"], start_new_session=True)
signal.signal(signal.SIGTERM, lambda *a: (open(marker, "w").write("cleaned"), sys.exit(0)))
time.sleep(300)
E

count() { pgrep -f "$1-grandchild" | wc -l | tr -d ' '; }
check() {  # name, tag, expect_marker
  sleep 4.5
  local n; n=$(count "$2")
  if [ "$n" != 0 ]; then echo "FAIL $1: $n survivor(s)"; fail=1; pkill -9 -f "$2-grandchild"; else echo "pass $1: no survivors"; fi
  if [ "$3" = yes ]; then
    if [ -f "$T/$2.marker" ]; then echo "pass $1: job's SIGTERM handler ran"; else echo "FAIL $1: no grace, handler never ran"; fail=1; fi
  fi
}

python3 tools/run-guarded.py --timeout 2 -- python3 "$T/job.py" rg-timeout "$T/rg-timeout.marker" 2>/dev/null
check timeout rg-timeout yes

python3 tools/run-guarded.py --timeout 60 -- python3 "$T/job.py" rg-term "$T/rg-term.marker" 2>/dev/null & P=$!
sleep 2; kill -TERM $P; wait $P 2>/dev/null
check sigterm-to-runner rg-term yes

python3 tools/run-guarded.py --timeout 60 -- python3 "$T/job.py" rg-kill "$T/rg-kill.marker" 2>/dev/null & P=$!
sleep 2; kill -KILL $P; wait $P 2>/dev/null
check sigkill-to-runner rg-kill yes

python3 tools/run-guarded.py --timeout 10 -- true; [ $? = 0 ] && echo "pass exit code passthrough" || { echo "FAIL exit code"; fail=1; }
exit $fail
