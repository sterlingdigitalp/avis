#!/usr/bin/env bash
# Run one prepared 2D or 3D case in the OpenFOAM container.
#   cfd/run-case.sh <caseDir> <containerSuffix> <cpus> <memory> <maxSeconds> [script-in-case]
#
# Limits are parameters, never hardcoded. The container is removed three ways, because a SIGKILL to this
# wrapper cannot run a trap: (1) a trap for normal exit and signals, (2) a detached sidecar that watches
# this wrapper's pid, (3) `timeout` inside the container, which bounds it even if the first two fail.
set -euo pipefail
CASE="$(cd "${1:?caseDir}" && pwd)"
NAME="avis-cfd-${2:?suffix}"
CPUS="${3:?cpus}"
MEM="${4:?memory, e.g. 16g}"
MAXSEC="${5:?max seconds}"
SCRIPT="${6:-.run.sh}"
IMAGE="${AVIS_CFD_IMAGE:-aerolab-openfoam:13}"

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
trap 'exit 129' HUP
cleanup

python3 -I -c '
import os,signal,subprocess,sys,time
os.setsid()  # leave the wrapper group: run-guarded SIGKILLs that whole group
for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT): signal.signal(s, signal.SIG_IGN)
pid,name=int(sys.argv[1]),sys.argv[2]
while True:
    time.sleep(2)
    try: os.kill(pid,0)
    except ProcessLookupError: break
subprocess.run(["docker","rm","-f",name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
' "$$" "$NAME" </dev/null >/dev/null 2>&1 &
disown $! 2>/dev/null || true

# Case data reaches the container through a bind mount at /work; PIDs stay inside.
docker run --rm --name "$NAME" --label avis.cfd=1 --cpus "$CPUS" --memory "$MEM" --memory-swap "$MEM" \
  -v "$CASE:/work" -w /work --entrypoint /bin/bash "$IMAGE" \
  -c "source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; timeout -k 20 $MAXSEC bash /work/$SCRIPT" </dev/null
