"""Owned Linux process trees, including children which change process groups.

This module has no ROS dependency so emergency cleanup also works after a ROS
bootstrap failure. PID start ticks prevent acting on a subsequently reused PID.
"""
from pathlib import Path
import argparse
import json
import os
import signal
import subprocess
import time


def process_identity(pid):
    try:
        text = Path('/proc', str(pid), 'stat').read_text()
        fields = text[text.rfind(')') + 2:].split()
        if fields[0] == 'Z':
            return None
        return dict(pid=int(pid), parent=int(fields[1]), group=int(fields[2]), start=fields[19])
    except (OSError, ValueError, IndexError):
        return None


def same_process(identity):
    current = process_identity(identity['pid'])
    return current is not None and current['start'] == str(identity['start'])


def process_table():
    return [process_identity(p.name) for p in Path('/proc').iterdir() if p.name.isdigit()]


def capture_tree(identities, candidates=None):
    known = {int(p['pid']): p for p in identities if same_process(p)}
    # A launch parent can exit before its children. Its live process group
    # still belongs to us; never infer ownership through a reused leader PID.
    groups = set()
    for p in identities:
        current = process_identity(p['pid'])
        if current is None or current['start'] == str(p['start']):
            groups.add(p.get('group', p['pid']))
    if candidates is None:
        candidates = process_table()
    while True:
        extra = [p for p in candidates if p and p['pid'] not in known and
                 (p['parent'] in known or p['group'] in groups)]
        if not extra:
            break
        known.update((p['pid'], p) for p in extra)
    return list(known.values())


def signal_tree(identities, sig):
    # Children first: a launch process may reap its children while exiting.
    for p in reversed(identities):
        if same_process(p):
            try:
                os.kill(p['pid'], sig)
            except ProcessLookupError:
                pass


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
    temporary.replace(path)


class OwnedProcess:
    def __init__(self, name, argv, cwd, output, popen=subprocess.Popen):
        self.name = name
        self.process = popen(argv, cwd=str(cwd), stdin=subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             start_new_session=True, text=True, encoding='utf-8', errors='replace')
        identity = process_identity(self.process.pid)
        self.identities = [identity] if identity else []
        self.stop_stage = 0
        self.unexpected_exits = []
        self.deadline = 0
        self.output = output

    def refresh(self, candidates=None):
        self.process.poll()  # Reap our direct child.
        previous = self.identities
        self.identities = capture_tree(previous, candidates)
        alive = {(p['pid'], p['start']) for p in self.identities}
        if not self.stop_stage:
            self.unexpected_exits.extend(p for p in previous if (p['pid'], p['start']) not in alive)

    def begin_stop(self, now):
        if self.stop_stage:
            return
        self.refresh()
        self.stop_stage = 1
        self.deadline = now + 2
        signal_tree(self.identities, signal.SIGINT)

    def tick_stop(self, now):
        self.refresh()
        if not self.identities:
            return True
        if now >= self.deadline:
            sig = signal.SIGTERM if self.stop_stage == 1 else signal.SIGKILL
            self.output('log', component=self.name,
                        message='Stopping timed out; sending ' + signal.Signals(sig).name)
            signal_tree(self.identities, sig)
            self.stop_stage += 1
            self.deadline = now + (2 if sig == signal.SIGTERM else 1)
        return False


def cleanup_registry(path):
    """Called by Qt only after its runtime helper exits unexpectedly."""
    path = Path(path)
    if not path.exists():
        return
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('manager') and same_process(data['manager']):
        raise RuntimeError('Runtime manager is still alive; refusing parallel cleanup')
    # First revoke the managed bridge lease. It independently stops at 20 Hz.
    atomic_json(path.parent / 'context.json', dict(permit=False))
    for name in ('follow', 'odom', 'chassis'):
        identities = capture_tree(data.get('owned', {}).get(name, []))
        for sig, timeout in ((signal.SIGINT, 2), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
            signal_tree(identities, sig)
            until = time.monotonic() + timeout
            while identities and time.monotonic() < until:
                time.sleep(.05)
                identities = capture_tree(identities)
            if not identities:
                break
        if identities:
            raise RuntimeError('Could not clean up owned ' + name + ' process tree')
    atomic_json(path, dict(manager=None, owned={}))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cleanup-registry', required=True)
    cleanup_registry(parser.parse_args().cleanup_registry)


if __name__ == '__main__':
    main()
