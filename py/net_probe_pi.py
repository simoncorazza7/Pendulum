"""Network latency probe, Pi side (no GPIO, nothing moves). Runs on the Pi, fed through ssh:

    ssh pendel@pendel.local "python3 - --seconds 600" < py/net_probe_pi.py > logs/<name>.csv

Waits for a hello from py/net_probe_pc.py, then sends a 24-byte packet every Ts (as the remote control loop
will: seq, send time) to the address the hello came from, and timestamps each echo on arrival (CLOCK_MONOTONIC,
the Pi's own clock, so no clock sync is needed). The round trip measured here is what a remote controller has:
sample leaves the Pi -> command back at the Pi. Output (stdout): '#' header lines, then 'seq,rtt_us' per packet
(rtt_us = -1: no echo by the end).
"""
import argparse
import select
import socket
import struct
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=9750)
ap.add_argument("--seconds", type=float, default=600.0)
ap.add_argument("--ts", type=float, default=0.002)
a = ap.parse_args()

PKT = struct.Struct("<IIq8x")             # magic, seq, t_send_ns, padding to 24 bytes (about the real sample size)
MAGIC = 0x50524F42                       # 'PROB'
s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
s.bind(("::", a.port))
print(f"# probe: waiting for a hello on udp {a.port}", file=sys.stderr, flush=True)
s.settimeout(120)
_, peer = s.recvfrom(64)
s.setblocking(False)
print(f"# peer {peer[0]} port {peer[1]}", file=sys.stderr, flush=True)

n = int(a.seconds / a.ts)
period = int(a.ts * 1e9)
t_send = [0] * n
rtt = [-1] * n
t0 = time.monotonic_ns() + 10_000_000
k = 0
while k < n or time.monotonic_ns() < t0 + n * period + 2_000_000_000:
    deadline = t0 + k * period
    while True:                           # wait for the next send deadline, timestamping echoes as they arrive
        now = time.monotonic_ns()
        if now >= deadline and k < n:
            break
        if k >= n and now >= t0 + n * period + 2_000_000_000:
            break
        r, _, _ = select.select([s], [], [], max(0.0, (deadline - now) / 1e9) if k < n else 0.05)
        if r:
            while True:
                try:
                    data, _ = s.recvfrom(64)
                except BlockingIOError:
                    break
                tr = time.monotonic_ns()
                if len(data) == PKT.size:
                    m, seq, ts_ = PKT.unpack(data)
                    if m == MAGIC and seq < n and rtt[seq] < 0 and t_send[seq] == ts_:
                        rtt[seq] = (tr - ts_) // 1000
    if k >= n:
        break
    t_send[k] = time.monotonic_ns()
    try:
        s.sendto(PKT.pack(MAGIC, k, t_send[k]), peer)
    except OSError:
        pass
    k += 1

late_send = sum(1 for i in range(1, n) if t_send[i] - (t0 + i * period) > 1_000_000)
print(f"# packets {n}, ts {a.ts}, sends more than 1 ms late {late_send}")
print("seq,rtt_us")
out = sys.stdout
for i in range(n):
    out.write(f"{i},{rtt[i]}\n")
