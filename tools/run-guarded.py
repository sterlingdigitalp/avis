#!/usr/bin/env python3
"""Run a command so that nothing it starts can outlive it.

    tools/run-guarded.py --timeout 300 [--gpu] --log out/x.log -- cmd args...

Process groups are not enough: Playwright starts Chromium detached, in its own group, so a group
kill misses it. So this tracks the whole descendant tree by pid and start time:

- A watcher process snapshots the tree every half second. Descendants that get reparented to
  launchd stay on its list, so they are still found after their parent dies.
- On timeout, on SIGTERM/SIGHUP/SIGINT, on normal exit, or if this script itself is SIGKILLed
  (the watcher notices), every tracked process gets SIGTERM, then SIGKILL after a grace period.
  The grace lets the job's own cleanup run, such as Playwright closing its browser.
- Start times are checked before every signal, so a reused pid is never hit.

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
GRACE_S = 3.0


def process_table():
    """{pid: (ppid, start)} for every process; start is ps's lstart text."""
    out = subprocess.run(["ps", "-axo", "pid=,ppid=,lstart="], capture_output=True, text=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3:
            table[int(parts[0])] = (int(parts[1]), parts[2].strip())
    return table


def tree(root, table):
    """{pid: start} for root and all its current descendants."""
    if root not in table:
        return {}
    kids = {}
    for pid, (ppid, _) in table.items():
        kids.setdefault(ppid, []).append(pid)
    found, stack = {}, [root]
    while stack:
        pid = stack.pop()
        found[pid] = table[pid][1]
        stack.extend(kids.get(pid, []))
    return found


def alive(known, table):
    return {pid: st for pid, st in known.items() if pid in table and table[pid][1] == st}


def signal_all(known, sig):
    for pid, st in alive(known, process_table()).items():
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def kill_tree(known, grace=GRACE_S):
    """SIGTERM, wait up to grace, SIGKILL the rest. Returns how many needed SIGKILL."""
    known = alive(known, process_table())
    if not known:
        return 0
    signal_all(known, signal.SIGTERM)
    deadline = time.time() + grace
    while time.time() < deadline:
        time.sleep(0.1)
        if not alive(known, process_table()):
            return 0
    left = alive(known, process_table())
    signal_all(left, signal.SIGKILL)
    return len(left)


def watch(parent, root):
    """Runs as its own process. Tracks root's tree; kills it if parent disappears."""
    known = {}
    while True:
        table = process_table()
        known.update(tree(root, table))
        known = alive(known, table)
        if parent not in table:
            kill_tree(known)
            return
        if not known:
            return
        time.sleep(0.5)


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


def main():
    a = parse()
    gpu_fd = take_gpu(a.gpu_wait) if a.gpu else None
    out = None
    if a.log:
        os.makedirs(os.path.dirname(os.path.abspath(a.log)), exist_ok=True)
        out = open(a.log, "w")
    child = subprocess.Popen(
        a.cmd, stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT if out else None,
        start_new_session=True,
    )
    watcher = subprocess.Popen(
        [sys.executable, "-I", os.path.abspath(__file__), "--_watch", str(os.getpid()), str(child.pid)],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    seen = {}

    def cleanup():
        seen.update(tree(child.pid, process_table()))
        kill_tree(seen)
        if watcher.poll() is None:
            watcher.kill()

    atexit.register(cleanup)

    def on_signal(signum, _):
        cleanup()
        sys.exit(128 + signum)

    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)
    t_end = time.time() + a.timeout
    rc = None
    while rc is None:
        seen.update(tree(child.pid, process_table()))
        try:
            rc = child.wait(min(1.0, max(0.0, t_end - time.time())))
        except subprocess.TimeoutExpired:
            if time.time() >= t_end:
                print(f"TIMEOUT after {a.timeout}s: {a.cmd[0]}", file=sys.stderr)
                cleanup()
                rc = 124
    if gpu_fd is not None:
        os.close(gpu_fd)
    sys.exit(rc)


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--_watch":
        watch(int(sys.argv[2]), int(sys.argv[3]))
    else:
        main()
