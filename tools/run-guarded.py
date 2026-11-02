#!/usr/bin/env python3
"""Run a command in its own process group so it cannot outlive this script.

    tools/run-guarded.py --timeout 300 [--gpu] --log out/x.log -- cmd args...

The group gets SIGKILL on timeout, on SIGTERM/SIGHUP/SIGINT to this script, and on normal exit.
The one case a parent cannot catch is its own SIGKILL, so a child watcher also polls this
process's pid and kills the group if it disappears.

--gpu takes an exclusive flock on ~/avis/.gpu.lock. The kernel releases it when the holder dies,
so a crashed job can never leave a stale lock.
"""
import argparse
import atexit
import fcntl
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse():
    p = argparse.ArgumentParser()
    p.add_argument("--timeout", type=int, required=True)
    p.add_argument("--gpu", action="store_true")
    p.add_argument("--gpu-wait", type=int, default=3600, help="max seconds to wait for the GPU lock")
    p.add_argument("--log", default=None)
    p.add_argument("cmd", nargs=argparse.REMAINDER)
    a = p.parse_args()
    if a.cmd[:1] == ["--"]:
        a.cmd = a.cmd[1:]
    if not a.cmd:
        p.error("no command")
    return a


def take_gpu(wait):
    fd = os.open(os.path.join(ROOT, ".gpu.lock"), os.O_CREAT | os.O_RDWR, 0o644)
    t0 = time.time()
    while True:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return fd
        except BlockingIOError:
            if time.time() - t0 > wait:
                print(f"GPU_LOCK_TIMEOUT after {wait}s", file=sys.stderr)
                sys.exit(75)
            time.sleep(2)


def start_watcher(pgid):
    """Kill the group if this script is SIGKILLed. Inherits nothing but two pids."""
    me = os.getpid()
    code = (
        "import os,time,signal\n"
        f"me,g={me},{pgid}\n"
        "while True:\n"
        "    time.sleep(1)\n"
        "    if os.getppid()!=me:\n"
        "        try: os.killpg(g,signal.SIGKILL)\n"
        "        except ProcessLookupError: pass\n"
        "        break\n"
        "    try: os.killpg(g,0)\n"
        "    except ProcessLookupError: break\n"
    )
    return subprocess.Popen(
        [sys.executable, "-I", "-c", code],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def main():
    a = parse()
    gpu_fd = take_gpu(a.gpu_wait) if a.gpu else None
    out = None
    if a.log:
        os.makedirs(os.path.dirname(os.path.abspath(a.log)), exist_ok=True)
        out = open(a.log, "w")
    child = subprocess.Popen(
        a.cmd, stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT if out else None,
        start_new_session=True, pass_fds=(),
    )
    watcher = start_watcher(child.pid)

    def kill_group():
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except PermissionError:
            pass
        watcher.poll() is None and watcher.kill()

    atexit.register(kill_group)

    def on_signal(signum, _):
        kill_group()
        sys.exit(128 + signum)

    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)
    try:
        rc = child.wait(a.timeout)
    except subprocess.TimeoutExpired:
        print(f"TIMEOUT after {a.timeout}s: {a.cmd[0]}", file=sys.stderr)
        kill_group()
        rc = 124
    if gpu_fd is not None:
        os.close(gpu_fd)
    sys.exit(rc)


if __name__ == "__main__":
    main()
