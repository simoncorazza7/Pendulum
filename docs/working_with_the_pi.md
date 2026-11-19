# Working with the Pendel code

How to get the code, read it, keep it current, and later work on the rig's Raspberry Pi. Nothing in parts 1
and 2 touches the rig.

## 1. Get the code (GitHub)

1. Install Git for Windows (git-scm.com). Use **PowerShell** for the commands below.
2. **Clone** (sign in to GitHub when Git asks):
   ```
   cd $HOME\Documents
   git clone https://github.com/simoncorazza7/Pendulum.git Pendel-main
   cd Pendel-main
   git switch control-dev
   ```
   `control-dev` is the current branch (on GitHub since 2026-11-19); GitHub's `main` is older and out of date.
3. **Stay current** whenever something new has been pushed: `git pull`.

## 2. Where to start reading

| File | What it is |
|---|---|
| `CLAUDE.md` | rules, conventions, hardware facts and the current status, condensed |
| `docs/explainer.md` | overview of the rig and how the balance control works |
| `docs/stage1_demo.md` | how a balance session runs, and the results so far |
| `config/pendctl.conf` | every setting, with units and reasons in the comments |
| `src/core/` | controller maths: PID, supervisor, estimator, units |
| `src/runtime/` | the loop logic: balance, session, swing-up, moves |
| `src/app/` | the programs on the Pi (`pendctl` modes) |
| `py/` | analysis and plotting on the PC; `py/remote_ctrl.py` is the controller-on-the-PC program |
| `tests/` | unit tests (`tests/unit`), golden data, mock and end-to-end tests |

To run the Python tools, install Python 3.14 (the version `py/requirements.txt` names; its pinned numpy and scipy need
3.12 or newer) and then:
`python -m venv .venv`, then `.venv\Scripts\python -m pip install -r py\requirements.txt`.

## 3. Later: working on the Pi

### Before anything: the rig moves

- **Checklist first, and know what will move:** anything that moves the cart or opens the real GPIO chip:
  - `./demo.sh`, `./swingup.sh`, `./remote.sh`, `./start.sh`, `build/main`;
  - `build/pendctl --mode move | damp | balance | shadow | encoder-log`.
- **Always safe (nothing moves):** `make`, `make test`, copying logs, reading files, `build/pendctl --mode log-selftest`.
- **When the rig is powered:** keep the motor power (rocker) switch within reach. Ctrl-C is a controlled stop.
- **After anything unexpected:** check that the driver is off. `pinctrl get 24` must show `op` `dh`;
  `pinctrl set 24 op dh` turns it off.

### Connect to the Pi (once)

Use PowerShell: Git Bash can't resolve `pendel.local`.

1. **Make Git use Windows' SSH, and make a key:**
   ```
   git config --global core.sshCommand "C:/Windows/System32/OpenSSH/ssh.exe"
   ssh-keygen -t ed25519 -C "your-name"
   ```
2. **Put the key on the Pi.** Type the Pi's password once:
   ```
   type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh pendel@pendel.local "mkdir -p ~/.ssh && chmod 700 ~/.ssh && tr -d '\r' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
   ```
   - Accept the fingerprint only if it is `SHA256:+/uYSPx4f31bInTJd3Lgdp3Fgh76LSaEVURRtlaBABE` (ED25519; the
     replacement Pi since 2026-10-23).
   - Check: `ssh pendel@pendel.local hostname` prints `pendel` without asking for a password.
3. **Add the Pi as a second place to push to:** `git remote add pi pendel@pendel.local:Pendel-main`.

### Look at files and logs on the Pi

- **Shell on the Pi:** `ssh pendel@pendel.local`, then `cd ~/Pendel-main`. Useful there:
  - `git log --oneline -5`: what is deployed;
  - `ls logs`: one folder per run;
  - `less config/pendctl.conf`: the settings.
- **Copy runs to your computer:** `.venv\Scripts\python py\fetch_runs.py --last 3` (into `logs\`), then
  `.venv\Scripts\python py\plot_run.py`.

The code on the Pi (`~/Pendel-main`, branch `control-dev`) is a git repository that accepts pushes straight into
its working files. **Never edit files on the Pi itself:** the next push would be refused, and nobody would know
what changed.

### Change something and send it to the Pi

1. `git pull`: start from the latest version.
2. Edit on your computer. To try a value for one run only, use an override instead of editing:
   `./demo.sh --set encoder.trim=-1.6`.
3. Check and commit: `git status`, `git diff`, `git add <file>`, `git commit -m "what changed and why"`.
4. `git push pi control-dev`: the Pi's files now match your commit. Push to GitHub only finished, tested work.
5. Build and test on the Pi; nothing moves: `ssh pendel@pendel.local "cd ~/Pendel-main && make && make test"`.
   The build deletes the old program first, so a failed build never leaves a stale one to run.
6. Confirm: `ssh pendel@pendel.local "cd ~/Pendel-main && git log --oneline -1"`.

### When something goes wrong

| You see | Meaning | Do |
|---|---|---|
| push rejected: `fetch first` / `non-fast-forward` | someone pushed since your pull | `git pull --rebase pi control-dev`, push again |
| push rejected: `Working directory has unstaged changes` | someone edited files on the Pi | stop; look at `git status` on the Pi first; don't force |
| `Could not resolve hostname pendel.local` | Pi off or elsewhere, or you're in Git Bash | use PowerShell; check cable and power; use the Pi's IP address |
| `Permission denied (publickey)` | your key isn't on the Pi | repeat "Put the key on the Pi" |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | another machine answers to that name, or your PC still remembers the Pi replaced on 2026-10-23 | stop; check the fingerprint on the Pi itself (`ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`), then `ssh-keygen -R pendel.local` |
| the build fails on the Pi | the pushed code doesn't compile | fix, commit, push again; never run an old build |

Never `git push --force`.

### Running the rig

`ssh -t pendel@pendel.local`, then `cd ~/Pendel-main && ./demo.sh` (or `./swingup.sh`, `./remote.sh`). Each starts
with a checklist. After every run: `pinctrl get 24` must show `op dh`.

**macOS / Linux:** use Terminal, skip the `core.sshCommand` line, put the key on the Pi with
`ssh-copy-id pendel@pendel.local`, and use `.venv/bin/python` with forward slashes.
