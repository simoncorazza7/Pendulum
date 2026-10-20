"""PC side of the remote controller (brief Stage 2b, docs/remote.md): runs the balance controller on the samples
the Pi streams over UDP and sends each command straight back.

    python py/remote_ctrl.py [--host pendel.local] [--port 9760] [--normal-priority] [--stay]

Start this first, then on the Pi either the rig (./remote.sh, HARDWARE) or the simulated pendulum
(build/pendctl --mode remote-sim, no GPIO). Needs only the Python standard library: py/pendel/control.py (the
Stage 1 PID, golden-tested against the Pi's C++) is loaded directly, without the analysis package.

How it fits together:
  - the Pi sends PARAMS (its gains and limits, so both sides compute the same PID) on HELLO and every 0.5 s,
    and a SAMPLE every 2 ms tick: seq, theta, omega (the Pi's estimator), x, v, and whether the supervisor is
    running the controller on it (ENGAGED);
  - this program answers every SAMPLE at once with a REPLY: the command computed from it (0 unless ENGAGED);
  - the Pi applies, at tick k, the command computed from the sample of tick k - delay_ticks: a CONSTANT delay
    whatever the network jitter, as long as every reply arrives within delay_ticks ticks. A late reply makes
    the Pi hold the previous command; remote.max_miss of those in a row abort the run. All safety stays on the Pi.
To try another controller, replace PidController (same three methods). Reacts to packets: no sleep() pacing.
Logs (on exit): logs/<time>_remote-pc/samples.csv and summary.json (PC receive intervals, processing times, lost
samples). The Pi logs the round trips (remote.csv); py/remote_report.py analyses both.
"""

from __future__ import annotations

import argparse
import collections
import gc
import importlib.util
import json
import pathlib
import socket
import struct
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("pendel_control", HERE / "pendel" / "control.py")
control = importlib.util.module_from_spec(_spec)
sys.modules["pendel_control"] = control           # dataclasses look their module up while defining the classes
_spec.loader.exec_module(control)

# --- protocol (src/common/remote_proto.h; golden bytes: tests/golden/remote_packets.txt) ---
VERSION = 1
MAGIC_HELLO, MAGIC_PARAMS, MAGIC_SAMPLE, MAGIC_REPLY = 0x4C454850, 0x4D525050, 0x504D5350, 0x444D4350
HELLO = struct.Struct("<IHH")                    # magic, version, 0                                   8 bytes
PARAMS = struct.Struct("<IHHI11d")               # magic, version, delay_ticks, params_id, ts, kp, ki, kd, a_max,
                                                 # v_max, v_taper, i_max, i_delay, theta_ref, vel_fc   100 bytes
SAMPLE = struct.Struct("<IHHIIIqiiddddBBH")      # magic, version, flags, seq, epoch, params_id, t_ns, counts,
                                                 # pos_usteps, theta, omega, x, v, mode, phase, delay  72 bytes
REPLY = struct.Struct("<IHHIIIdII")              # magic, version, flags, seq, epoch, params_id, a_cmd,
                                                 # pc_proc_ns, 0                                        36 bytes
SAMPLE_CONTROLLER, SAMPLE_FINISHING = 1, 2
REPLY_PARAMS_OK, REPLY_COMPUTED = 1, 2
MODES = {0: "IDLE", 1: "ARMED", 2: "ENGAGED", 3: "STOPPING", 255: "-"}
PHASES = {0: "CALIBRATE", 1: "WAIT_GO", 2: "HOME", 3: "BALANCE", 4: "AFTER_RUN", 5: "RECENTRE", 6: "FINISHING",
          7: "FINISHED", 8: "FAILED"}
assert (HELLO.size, PARAMS.size, SAMPLE.size, REPLY.size) == (8, 100, 72, 36)


class PidController:
    """The Stage 1 PID with the Pi's parameters (identical to the Pi's shadow PID on the same samples)."""

    def __init__(self, prm: dict):
        self.ts = prm["ts"]
        self.p = control.PidParams(kp=prm["kp"], ki=prm["ki"], kd=prm["kd"], a_max=prm["a_max"], v_max=prm["v_max"],
                                   v_taper=prm["v_taper"], i_max=prm["i_max"], i_delay=prm["i_delay"],
                                   theta_ref=prm["theta_ref"], vel_fc=prm["vel_fc"])
        self.reset()

    def reset(self) -> None:                       # a new engagement (epoch)
        self.s = control.PidState()

    def step(self, theta: float, omega: float, x: float, v: float) -> float:   # m/s^2
        a, self.s, _ = control.pid_step(self.p, self.s, theta, v, self.ts, omega)
        return a


def golden_lines() -> list[str]:
    """The packets test_remote.cpp packs (same values), as hex."""
    hello = HELLO.pack(MAGIC_HELLO, VERSION, 0)
    params = PARAMS.pack(MAGIC_PARAMS, VERSION, 3, 0x12345678, 0.002, 62.78, 96.75, 6.556, 2.0, 0.3, 0.25, 1.0, 0.0,
                         0.0, 14.0)
    sample = SAMPLE.pack(MAGIC_SAMPLE, VERSION, 1, 123456, 7, 0x12345678, 1234567890123, -4797, 6104, 0.0123, -0.456,
                         -0.0789, 0.101, 2, 3, 3)
    reply = REPLY.pack(MAGIC_REPLY, VERSION, 3, 123453, 7, 0x12345678, -1.25, 54321, 0)
    return [f"hello {hello.hex()}", f"params {params.hex()}", f"sample {sample.hex()}", f"reply {reply.hex()}"]


def resolve(host: str, port: int):
    infos = socket.getaddrinfo(host, port, socket.AF_UNSPEC, socket.SOCK_DGRAM)
    infos.sort(key=lambda i: 0 if i[0] == socket.AF_INET else 1)     # IPv4 first (no link-local scope ids)
    return infos[0][0], infos[0][4]


def high_priority() -> str:
    """This process only, undone when it exits: Windows HIGH_PRIORITY_CLASS + a TIME_CRITICAL thread (normal
    programs can no longer take the CPU for a whole ~16 ms scheduler slice) + a 1 ms timer resolution request."""
    if sys.platform == "win32":
        import atexit
        import ctypes
        from ctypes import wintypes
        k = ctypes.WinDLL("kernel32")
        # 64-bit: without these the pseudo-handles are passed as 32-bit ints, an invalid handle, and the calls fail
        k.GetCurrentProcess.restype = wintypes.HANDLE
        k.GetCurrentThread.restype = wintypes.HANDLE
        k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        k.SetPriorityClass.restype = wintypes.BOOL
        k.SetThreadPriority.argtypes = [wintypes.HANDLE, ctypes.c_int]
        k.SetThreadPriority.restype = wintypes.BOOL
        got = []
        if k.SetPriorityClass(k.GetCurrentProcess(), 0x00000080):      # HIGH_PRIORITY_CLASS
            got.append("high priority class")
        if k.SetThreadPriority(k.GetCurrentThread(), 15):              # THREAD_PRIORITY_TIME_CRITICAL
            got.append("time-critical thread")
        try:
            w = ctypes.windll.winmm
            if w.timeBeginPeriod(1) == 0:                              # TIMERR_NOERROR
                atexit.register(w.timeEndPeriod, 1)
                got.append("1 ms timer")
        except OSError:
            pass
        return ", ".join(got) if got else "priority unchanged"
    try:
        import os
        os.nice(-10)
        return "nice -10"
    except OSError:
        return "priority unchanged (nice needs root)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="pendel.local", help="the Pi")
    ap.add_argument("--port", type=int, default=9760, help="remote.port on the Pi")
    ap.add_argument("--log-dir", default=str(HERE.parent / "logs"))
    ap.add_argument("--normal-priority", action="store_true",
                    help="don't raise this process's priority (by default: Windows high priority class, time-critical "
                         "thread, 1 ms timer; this process only. Laptop test: worst answer 15.9 ms -> 0.99 ms)")
    ap.add_argument("--stay", action="store_true", help="keep waiting for the next session after one ends")
    ap.add_argument("--idle-exit", type=float, default=10.0, help="s without samples (after the first) before ending")
    ap.add_argument("--golden", action="store_true", help="print the golden packets (tests/golden/remote_packets.txt)")
    a = ap.parse_args()
    if a.golden:
        print("\n".join(golden_lines()))
        return 0

    fam, addr = resolve(a.host, a.port)
    sock = socket.socket(fam, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xB8)        # DSCP EF; ignored where unsupported
    except OSError:
        pass
    sock.settimeout(0.2)
    print(f"remote_ctrl: PC controller for {a.host} {addr[0]}:{a.port}" +
          (f" ({high_priority()})" if not a.normal_priority else " (normal priority)"))
    print("remote_ctrl: waiting for the Pi (./remote.sh, or build/pendctl --mode remote-sim). Ctrl-C ends.", flush=True)

    hello = HELLO.pack(MAGIC_HELLO, VERSION, 0)
    ctrl = None
    params_id = None
    epoch = None
    peer = None
    last_seq = None
    lost = n_samples = n_engaged = 0
    t_last_rx = t_last_hello = 0.0
    finishing_since = None
    next_status = 0.0
    proc_max = 0
    rows = collections.deque(maxlen=900 * 500)    # newest 15 min: seq, t_rx_ns, proc_ns, flags, a, theta
    t0 = time.perf_counter()
    gc.disable()
    try:
        while True:
            now = time.perf_counter()
            if ctrl is None or now - t_last_rx > 1.0:  # (re)connect: HELLO until PARAMS and samples flow
                if now - t_last_hello > 0.5:
                    try:
                        sock.sendto(hello, addr)
                    except OSError as e:
                        print(f"remote_ctrl: HELLO not sent: {e}", flush=True)
                    t_last_hello = now
            if finishing_since is not None and now - t_last_rx > 2.0 and not a.stay:
                print("remote_ctrl: the Pi ended the session", flush=True)
                break
            if n_samples and now - t_last_rx > a.idle_exit and not a.stay:
                print(f"remote_ctrl: no samples for {a.idle_exit:.0f} s; ending", flush=True)
                break
            try:
                data, src = sock.recvfrom(256)
            except socket.timeout:
                continue
            except (ConnectionResetError, OSError):  # Windows: ICMP port unreachable from an early HELLO
                continue
            t_rx = time.perf_counter_ns()
            t_last_rx = time.perf_counter()
            if len(data) < 8:
                continue
            magic, ver = struct.unpack_from("<IH", data)
            if ver != VERSION:
                continue
            if magic == MAGIC_PARAMS and len(data) == PARAMS.size:
                f = PARAMS.unpack(data)
                if f[3] != params_id:
                    keys = ("ts", "kp", "ki", "kd", "a_max", "v_max", "v_taper", "i_max", "i_delay", "theta_ref", "vel_fc")
                    prm = dict(zip(keys, f[4:]))
                    ctrl, params_id, epoch, peer = PidController(prm), f[3], None, src
                    print(f"remote_ctrl: Pi {src[0]} params: Kp {prm['kp']:.2f} Ki {prm['ki']:.2f} Kd {prm['kd']:.3f}, "
                          f"a_max {prm['a_max']:.1f} m/s^2, v_max {prm['v_max']:.2f} m/s; constant delay {f[2]} ticks = "
                          f"{f[2] * prm['ts'] * 1e3:.1f} ms", flush=True)
                continue
            if magic != MAGIC_SAMPLE or len(data) != SAMPLE.size:
                continue
            (_, _, flags, seq, ep, pid_id, t_ns, counts, pos, theta, omega, x, v, mode, phase,
             delay) = SAMPLE.unpack(data)
            n_samples += 1
            if last_seq is not None and seq > last_seq + 1:
                lost += seq - last_seq - 1
            last_seq = seq
            rflags, acmd = 0, 0.0
            if ctrl is not None and pid_id == params_id:
                rflags = REPLY_PARAMS_OK
                if ep != epoch:
                    ctrl.reset()
                    epoch = ep
                if flags & SAMPLE_CONTROLLER:
                    acmd = ctrl.step(theta, omega, x, v)
                    rflags |= REPLY_COMPUTED
                    n_engaged += 1
            proc = time.perf_counter_ns() - t_rx
            try:
                sock.sendto(REPLY.pack(MAGIC_REPLY, VERSION, rflags, seq, ep, pid_id, acmd, min(proc, 0xFFFFFFFF), 0), src)
            except OSError:
                pass
            proc_max = max(proc_max, proc)
            rows.append((seq, t_rx, proc, flags, acmd, theta))
            if flags & SAMPLE_FINISHING and finishing_since is None:
                finishing_since = time.perf_counter()
            if time.perf_counter() >= next_status:
                next_status = time.perf_counter() + (1.0 if mode == 2 else 5.0)
                print(f"{time.perf_counter() - t0:8.1f} s  Pi {PHASES.get(phase, phase)}/{MODES.get(mode, mode)}  "
                      f"theta {theta * 57.29578:+7.2f} deg  x {x * 1e3:+6.1f} mm  a {acmd:+5.2f}  | samples {n_samples}, "
                      f"lost {lost}, PC processing max {proc_max / 1e3:.0f} us", flush=True)
                proc_max = 0
    except KeyboardInterrupt:
        print("\nremote_ctrl: stopped (Ctrl-C)")
    finally:
        gc.enable()
        write_logs(pathlib.Path(a.log_dir), rows, lost, n_samples, n_engaged, peer, a)
    return 0


def write_logs(log_dir: pathlib.Path, rows, lost, n_samples, n_engaged, peer, a) -> None:
    if not rows:
        print("remote_ctrl: no samples received; no log")
        return
    d = log_dir / time.strftime("%Y-%m-%dT%H-%M-%S_remote-pc")
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "samples.csv", "w", newline="") as f:
        f.write("seq,t_rx_ns,proc_ns,flags,a_cmd,theta\n")
        for r in rows:
            f.write(f"{r[0]},{r[1]},{r[2]},{r[3]},{r[4]:.9g},{r[5]:.7f}\n")
    procs = sorted(r[2] for r in rows)
    gaps = sorted(rows[i][1] - rows[i - 1][1] for i in range(1, len(rows)) if rows[i][0] == rows[i - 1][0] + 1)
    q = lambda xs, p: xs[min(len(xs) - 1, int(p * (len(xs) - 1)))] / 1e3 if xs else None   # noqa: E731  (us)
    summary = {"host": a.host, "port": a.port, "peer": str(peer), "samples": n_samples, "engaged_samples": n_engaged,
               "lost_samples": lost, "logged": len(rows), "platform": sys.platform, "python": sys.version.split()[0],
               "pc_proc_us": {"p50": q(procs, 0.5), "p99": q(procs, 0.99), "p999": q(procs, 0.999), "max": q(procs, 1.0)},
               "pc_rx_interval_us": {"p50": q(gaps, 0.5), "p01": q(gaps, 0.001), "p999": q(gaps, 0.999),
                                     "max": q(gaps, 1.0)}}
    (d / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"remote_ctrl: {n_samples} samples ({n_engaged} engaged), {lost} lost; PC processing p99 "
          f"{summary['pc_proc_us']['p99']:.0f} us, max {summary['pc_proc_us']['max']:.0f} us\n  -> {d}")


if __name__ == "__main__":
    sys.exit(main())
