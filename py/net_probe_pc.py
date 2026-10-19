"""Network latency probe, PC side: echoes every packet from py/net_probe_pi.py straight back.

    .venv/Scripts/python py/net_probe_pc.py [--host pendel.local] [--port 9750]

Start this first; it sends a hello every 0.1 s until the Pi's packets arrive, then echoes until they stop for 3 s.
Reacts to incoming packets (no Sleep() pacing); the garbage collector is off while echoing.
"""
import argparse
import gc
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="pendel.local")
ap.add_argument("--port", type=int, default=9750)
a = ap.parse_args()

addr = socket.getaddrinfo(a.host, a.port, socket.AF_INET, socket.SOCK_DGRAM)[0][4]   # IPv4: the Pi binds dual-stack
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(0.1)
print(f"probe echo: hello to {addr}", flush=True)
n = 0
t_last = None
gc.disable()
while True:
    try:
        data, src = s.recvfrom(64)
    except socket.timeout:
        if n == 0:
            s.sendto(b"hello", addr)
            continue
        if time.monotonic() - t_last > 3.0:
            break
        continue
    except ConnectionResetError:          # Windows: ICMP port unreachable from an earlier hello
        continue
    try:
        s.sendto(data, src)
    except OSError as e:                  # e.g. the network went away for a moment: keep echoing when it's back
        print(f"probe echo: send failed ({e})", flush=True)
        continue
    n += 1
    t_last = time.monotonic()
gc.enable()
print(f"probe echo: echoed {n} packets", flush=True)
