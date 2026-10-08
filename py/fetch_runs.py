"""Copy run directories from the Pi (~/Pendel-main/logs) into ./logs on this PC.

Usage (from the repo root):
    .venv/Scripts/python py/fetch_runs.py            newest run
    .venv/Scripts/python py/fetch_runs.py --last 3   newest three runs
    .venv/Scripts/python py/fetch_runs.py NAME ...   specific run directories
Runs that already exist locally are skipped. Uses ssh/scp with key auth (BatchMode).
"""

import argparse
import pathlib
import subprocess

HOST = "pendel@pendel.local"
REMOTE_LOGS = "Pendel-main/logs"
SSH_OPTS = ["-o", "BatchMode=yes"]


def remote_runs() -> list[str]:
    out = subprocess.run(["ssh", *SSH_OPTS, HOST, f"ls -1 {REMOTE_LOGS}"],
                         check=True, capture_output=True, text=True).stdout
    return sorted(line.strip() for line in out.splitlines() if line.strip())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*")
    ap.add_argument("--last", type=int, default=1)
    args = ap.parse_args()

    names = args.names or remote_runs()[-args.last:]
    local = pathlib.Path("logs")
    local.mkdir(exist_ok=True)
    for name in names:
        if (local / name).exists():
            print(f"skip {name} (exists)")
            continue
        subprocess.run(["scp", "-q", "-r", *SSH_OPTS, f"{HOST}:{REMOTE_LOGS}/{name}", str(local) + "/"],
                       check=True)
        print(f"fetched {name}")


if __name__ == "__main__":
    main()
