#!/usr/bin/env python3
"""Send protocol lines to main's TCP server (127.0.0.1:8080) and print the raw reply.

Each argument is sent as one line ("" sends an empty line). Collects whatever arrives
within ~1.5 s, then closes. Test helper only.
"""
import socket
import sys
import time


def main() -> None:
    payload = "".join(line + "\n" for line in sys.argv[1:]).encode()
    sock = socket.create_connection(("127.0.0.1", 8080), timeout=2.0)
    sock.settimeout(0.3)
    sock.sendall(payload)
    reply = b""
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            if reply.count(b"\n") >= len(sys.argv) - 1:
                break
            continue
        if not chunk:
            break
        reply += chunk
    sock.close()
    sys.stdout.write(reply.decode(errors="replace"))


if __name__ == "__main__":
    main()
