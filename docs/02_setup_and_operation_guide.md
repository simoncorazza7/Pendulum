# Pendel: setup and operation guide

A step-by-step guide from an empty Windows PC to running the rig, in both modes:

- **local:** the controller runs on the Raspberry Pi;
- **remote:** the controller runs on your PC.

It assumes no prior experience with git, Python or a terminal. Windows is the main platform; macOS and Linux notes
appear where the steps differ.

- **Theory, hardware, software design, all symbols:** [`docs/01_theory_and_technical_reference.md`](01_theory_and_technical_reference.md).
  This guide uses its notation (θ, $x$, $a$, $K_p$, $N$, …) and links to its sections as "reference §n".
- **What this describes:** branch `control-dev` at commit `fad2469` (2026-11-04).

> ⚠️ TODO / unverified: callouts like this one mark steps the repo does not settle. Check them
> before relying on them.

## Safety first (read before anything else)

The rig is a moving machine with a belt, pulleys and a rod that can swing through a full circle.

1. **Nothing that can move the cart runs before the pre-flight checklist (§7.2), and before everyone near the rig
   knows what will move.** That covers `./demo.sh`, `./remote.sh`, `./swingup.sh`, `./pcbtest.sh E`, `./start.sh`,
   `build/main`, and the `pendctl` modes `move`, `damp` and `balance`. The modes `shadow` and `encoder-log` cannot
   move anything but still read the real GPIO chip. Building, testing and reading files never move anything (§7.1).
2. **Know where the rocker switch is.** It cuts the motor *and* the Pi at once: the hard emergency stop.
3. **Never put a multimeter probe on the Pi's 40-pin header while anything is powered.** A probe that slipped
   between pins 1 and 2 destroyed the rig's Pi on 2026-10-22. Use the test points in `docs/pcb_bringup.md`.
4. **Hands on the rod only, never near the belt or pulleys.** During swing-up, stay out of the pendulum's circle.
5. **After anything unexpected,** check that the motor driver is off (§7.5).

## How to read the commands

Every command block names its shell in the fence and the machine in its first line:

```powershell
# PC (PowerShell)
git --version
```

```bash
# Pi (bash, in an SSH session)
cd ~/Pendel-main
```

- Replace anything in angle brackets, like `<run folder>`, and leave out the brackets.
- Lines starting with `#` are comments; typing them does nothing.
- "The repo folder" means the folder you clone in §4 (`Pendel-main`).
- Run the commands one line at a time unless the step says otherwise.

**Contents**

0. [What you need](#0-what-you-need)
1. [Install git](#1-install-git)
2. [Install Python](#2-install-python)
3. [Optional tools](#3-optional-tools)
4. [Getting the code](#4-getting-the-code)
5. [The Python environment](#5-the-python-environment)
6. [The Pi: power, network, access](#6-the-pi-power-network-access)
7. [Running the local system](#7-running-the-local-system-controller-on-the-pi)
8. [Running the remote system](#8-running-the-remote-system-controller-on-the-pc)
9. [Changing gains and settings](#9-changing-gains-and-settings)
10. [Troubleshooting](#10-troubleshooting)
11. [Quick-reference cheat sheet](#11-quick-reference-cheat-sheet)

---

## 0. What you need

- A Windows 10 or 11 PC (macOS or Linux works too), with an **Ethernet cable** to the lab network for remote mode.
- A **GitHub account** with access to the repository, which is private.
- The **Pi's password** for your first login. It is not written down anywhere in the repo, and should not be.
- For any run on the rig: the rig in front of you, with its rocker switch within reach.

---

## 1. Install git

Git is the version-control tool: it downloads the code, keeps its history, and moves your changes to the Pi.

**Windows**

1. Download **Git for Windows** from <https://git-scm.com/downloads/win> and run the installer.
2. Most screens can stay at their defaults. The ones that matter:

   | Installer screen | Choose | Why |
   |---|---|---|
   | Choosing the default editor | an editor you know, e.g. Visual Studio Code or Notepad | the default is Vim, which is hard to leave if you have never used it (type `:q` and Enter) |
   | Adjusting your PATH environment | **Git from the command line and also from 3rd-party software** (recommended) | PowerShell and VS Code can then find `git` |
   | Choosing the SSH executable | keep the default, **Use bundled OpenSSH** | §6.3 points git at Windows' own SSH with one setting, because of how `pendel.local` resolves here |
   | Configuring the line ending conversions | keep the default | the repo's `.gitattributes` (`* text=auto eol=lf`) forces Unix line endings for its files either way, which the Pi needs |
   | Choose a credential helper | **Git Credential Manager** (default) | lets you sign in to GitHub in a browser window instead of typing passwords |

3. Open a **new** PowerShell window (Start menu → type "PowerShell") and check:

   ```powershell
   # PC (PowerShell)
   git --version
   ```

   Expected: a line like `git version 2.x.y.windows.z`. If PowerShell says it doesn't know `git`, close and reopen
   the window; if that doesn't help, re-run the installer and check the PATH screen.

4. Tell git who you are. Every commit records this. Use the e-mail address of your GitHub account:

   ```powershell
   # PC (PowerShell)
   git config --global user.name "Your Name"
   git config --global user.email "you@example.com"
   ```

**macOS:** typing `git --version` in Terminal offers to install the Xcode command-line tools, which include git.
**Linux (Debian/Ubuntu):** `sudo apt install git`. The identity commands in step 4 are the same.

---

## 2. Install Python

**Which version: Python 3.14.**

- It is the version `py/requirements.txt` names, and the one the existing analysis environment was built with (3.14.6).
- The pinned analysis packages, numpy 2.5.3 and scipy 1.18.1, require at least Python 3.12. They will not install
  on anything older.
- The remote-mode PC program, `py/remote_ctrl.py`, needs only Python's standard library, so even an older Python 3
  runs it (the e2e test runs it on the Pi's Python 3.11).

**Windows, recommended: the Python install manager.** Since Python 3.14, python.org recommends this tool. It
provides the commands `py`, `python` and `pymanager`.

1. Install "Python install manager" from the Microsoft Store, or download it from <https://www.python.org/downloads/>.
2. In a new PowerShell window, install 3.14 and list what is installed:

   ```powershell
   # PC (PowerShell)
   py install 3.14
   py list
   ```

3. On the first runtime install it may offer to add a directory (`%LocalAppData%\Python\bin`) to your PATH. Accept;
   it makes the plain `python` command work everywhere.

**Windows, alternative: the classic installer.** It is deprecated since 3.14 and will not be produced for 3.16 or
later. Download the 64-bit Windows installer for 3.14 from python.org. On its first screen, **tick "Add python.exe to
PATH"** at the bottom before clicking "Install Now". Without it, `python` is unknown in PowerShell.

**Check the install** (in a new PowerShell window):

```powershell
# PC (PowerShell)
py -3.14 --version
python --version
```

Both should print `Python 3.14.x`.

**`py` versus `python` on Windows.**

- `py` is the Python **launcher**. It knows every installed version and picks one: `py -3.14 script.py` runs
  exactly 3.14.
- `python` is whichever `python.exe` comes first on your PATH.
- When several versions are installed, `py -3.14` is the unambiguous choice. This guide uses `py -3.14` only to
  create the virtual environment (§5). After that, everything runs through the environment's own `python.exe`, so
  the question no longer matters.
- If typing `python` **opens the Microsoft Store**, that is a Windows placeholder for a missing Python. Install
  Python as above, or open Settings, search for "Manage app execution aliases", and check the Python entries.

**macOS:** the python.org installer, or `brew install python@3.14`. **Linux:** your distribution's packages or
pyenv. On both, the command is `python3`, not `py`.

---

## 3. Optional tools

**Visual Studio Code** (<https://code.visualstudio.com/>) with the **Python** extension from Microsoft.

- Open the repo folder with File → Open Folder.
- Choose the environment from §5 with Ctrl+Shift+P → "Python: Select Interpreter" → the one in `.venv`.
- VS Code's Markdown preview (Ctrl+Shift+V) shows the equations in these documents. The Mermaid diagrams need an
  extension, or view the files on GitHub, which renders both.

**Windows Terminal and PowerShell.** Windows 11 ships Windows Terminal; each tab is a PowerShell by default. The
few commands you need:

| Task | PowerShell | Notes |
|---|---|---|
| Where am I? | `pwd` | prints the current folder |
| Go into a folder | `cd Pendel-main`, `cd $HOME\Documents` | `$HOME` is your user folder |
| Go up one level | `cd ..` | |
| List files | `ls` or `dir` | both work in PowerShell |
| Paths with spaces | `cd "C:\My Folder"` | put them in quotes |
| Complete a name | Tab | type the first letters, press Tab |
| Repeat a command | Up arrow | |
| Stop a running program | Ctrl+C | |

- PowerShell accepts both `\` and `/` in most paths.
- Use **PowerShell, not Git Bash**, for anything that talks to `pendel.local`: on this setup Git Bash's tools
  cannot resolve that name (`docs/working_with_the_pi.md`).

---

## 4. Getting the code

### 4.1 The few git words you need

| Word | Meaning |
|---|---|
| repository (repo) | a project folder plus its complete history |
| clone | download a repository, history included, to your computer |
| commit | a saved snapshot of changes, with a message saying what and why |
| branch | a named line of development. Work here happens on `control-dev`. |
| remote | a named copy of the repo elsewhere: `origin` = GitHub, `pi` = the rig's Pi |
| pull | fetch new commits from a remote and merge them into your branch |
| push | send your commits to a remote |

### 4.2 Where the code is

The repository's GitHub address, from its configured remote (`git remote -v`):

```
https://github.com/simoncorazza7/Pendulum.git
```

> ⚠️ TODO / unverified: **GitHub does not have the current code.** As of 2026-11-18, GitHub has only the branch
> `main` (`5da1513`). The working branch `control-dev`, which everything in these documents describes, exists on
> the development PC and on the Pi, not on GitHub.
>
> GitHub's `main` is also inconsistent with itself: its Pi code speaks remote protocol version 1, but its
> `py/remote_ctrl.py` speaks version 4, so remote mode would never link (§8).
>
> Until `control-dev` is pushed to GitHub, the `git switch control-dev` step below fails with "invalid
> reference". Until then, clone the Pi's copy instead, which is what the rig runs (this needs the SSH access of
> §6): `git clone pendel@pendel.local:Pendel-main Pendel-main`.

### 4.3 Authentication

The repository is private: an anonymous download is refused. Use one of these:

**A. HTTPS with Git Credential Manager (simplest on Windows).** Just run the `git clone` below. The first time, a
window asks you to sign in to GitHub in your browser. Git then stores the credential in Windows' Credential
Manager, and later commands do not ask again.

**B. HTTPS with a personal access token.** On GitHub: your picture → Settings → Developer settings → Personal
access tokens. When git asks for a password, paste the token instead.

> ⚠️ TODO / unverified: GitHub documents gaps for *fine-grained* tokens on repositories where you are only a
> collaborator, which is the case for a repository owned by a personal account. A *classic* token with the
> `repo` scope is the usual alternative. Neither has been tested with this repository; prefer A or C.

**C. SSH keys.** Useful if you will also use a key for the Pi (§6.3). Create a key, show the public half, and add
it on GitHub under Settings → SSH and GPG keys → New SSH key:

```powershell
# PC (PowerShell)
ssh-keygen -t ed25519 -C "you@example.com"
type $env:USERPROFILE\.ssh\id_ed25519.pub
ssh -T git@github.com
```

- `ssh-keygen` asks where to save the key (press Enter for the default) and for a passphrase (recommended).
- Copy the whole line that `type` prints into GitHub's "Key" field.
- `ssh -T` should greet you by your GitHub user name.
- With SSH, clone `git@github.com:simoncorazza7/Pendulum.git` instead of the https address.

### 4.4 Clone

These are the commands from `docs/working_with_the_pi.md`. The folder name `Pendel-main` matches what the Pi and the
scripts use.

```powershell
# PC (PowerShell)
cd $HOME\Documents
git clone https://github.com/simoncorazza7/Pendulum.git Pendel-main
cd Pendel-main
git switch control-dev
git status
```

`git status` should say `On branch control-dev` and `nothing to commit, working tree clean`. (See the TODO in
§4.2 about `git switch control-dev`.)

**Later, to get the newest version:**

```powershell
# PC (PowerShell, in the repo folder)
git pull
```

Do this before you start any work. `git log --oneline -5` shows the last five commits.

---

## 5. The Python environment

A **virtual environment** ("venv") is a private copy of Python inside the repo folder, with exactly the package
versions the project pins. Nothing you install there affects other projects. `.venv/` is in `.gitignore`, so it is
never committed.

**Create it and install the packages** (commands from `py/requirements.txt` and `docs/working_with_the_pi.md`):

```powershell
# PC (PowerShell, in the repo folder)
py -3.14 -m venv .venv
.venv\Scripts\python -m pip install -r py\requirements.txt
.venv\Scripts\python -c "import numpy, scipy, matplotlib; print('ok')"
```

The last line prints `ok` when everything is installed. The pinned versions are numpy 2.5.3, scipy 1.18.1 and
matplotlib 3.11.2.

**You do not have to "activate" the environment.** Every command in this guide calls the environment's own
interpreter, `.venv\Scripts\python`, as the repo's documents do. If you prefer to activate it (so that `python`
means the venv's Python):

| Shell | Activate | Leave |
|---|---|---|
| PowerShell | `.venv\Scripts\Activate.ps1` | `deactivate` |
| cmd | `.venv\Scripts\activate.bat` | `deactivate` |
| Git Bash | `source .venv/Scripts/activate` | `deactivate` |
| macOS / Linux | `source .venv/bin/activate` | `deactivate` |

If PowerShell refuses with *"running scripts is disabled on this system"*, allow locally created scripts for your
user account only, then try again:

```powershell
# PC (PowerShell)
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

Or skip activation; nothing in this guide needs it.

**macOS / Linux:** `python3 -m venv .venv`, then `.venv/bin/python -m pip install -r py/requirements.txt`. In every
later command, write `.venv/bin/python` and forward slashes.

**The Pi does not use this environment.** It runs only standard-library Python scripts with its system `python3`
(3.11). Analysis and plotting happen on the PC.

> ⚠️ TODO / unverified: the C++ unit tests can also run on the PC (`bash tests/unit/run_pc.sh`), but that needs
> `clang++` with the Microsoft C++ standard library, and the repo does not document how it was installed. On the Pi,
> `make test` runs the same tests (§6.6).

---

## 6. The Pi: power, network, access

The rig's Raspberry Pi 4 is called `pendel`, its user is `pendel`, and its code lives in `~/Pendel-main` (`~` is its
home folder).

### 6.1 Power and network

- **Power.** The Pi gets its 5 V from the rig's 24 V supply through the carrier PCB, so the rocker switch powers the
  rig and the Pi together. Give it about a minute to boot.
  - For about the first second after power-on the motor driver is enabled and the motor holds, before the boot
    settings switch it off (`docs/pcb_bringup.md`, review finding 1). Nothing moves; this is expected.
  - Before switching on: nothing on the track, nobody touching the belt.
- **Network.** The Pi is wired (Ethernet) to the lab network and gets its address from DHCP. Your PC must be on the
  same network: wired for remote mode (§8), Wi-Fi is fine for logging in.

### 6.2 First login with SSH

SSH opens a terminal on the Pi from your PC.

```powershell
# PC (PowerShell)
ssh pendel@pendel.local
```

1. The first time, SSH shows the Pi's **host key fingerprint** and asks whether to continue. The replacement Pi
   fitted on 2026-10-23 has the ED25519 fingerprint `SHA256:+/uYSPx4f31bInTJd3Lgdp3Fgh76LSaEVURRtlaBABE`
   (`CLAUDE.md`). Type `yes` only if it matches.
2. Type the Pi's password. Nothing appears while you type; that is normal.
3. The prompt changes to `pendel@pendel:~ $`. You are now on the Pi. `exit` returns to your PC.

The same fingerprint is recorded in `docs/working_with_the_pi.md`. To check it on the Pi itself, run
`ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub` there (the check `setup_pi.sh` suggests).

### 6.3 Key-based login (once)

`py/fetch_runs.py` copies logs with `ssh -o BatchMode=yes`, which never asks for a password, so it only works with
a key. These are the steps from `docs/working_with_the_pi.md`:

```powershell
# PC (PowerShell)
git config --global core.sshCommand "C:/Windows/System32/OpenSSH/ssh.exe"
ssh-keygen -t ed25519 -C "your-name"
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh pendel@pendel.local "mkdir -p ~/.ssh && chmod 700 ~/.ssh && tr -d '\r' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
ssh pendel@pendel.local hostname
```

1. The first line makes git use Windows' own SSH, which can resolve `pendel.local`.
2. Skip `ssh-keygen` if you already made a key in §4.3; it would offer to overwrite it (answer `n`).
3. The third line asks for the Pi's password one last time. It appends your public key to the Pi's list of allowed
   keys; `tr -d '\r'` strips the Windows line ending.
4. The last line must print `pendel` **without** asking for a password.

**macOS / Linux:** skip the `core.sshCommand` line, and use `ssh-copy-id pendel@pendel.local` instead of the long
third line.

### 6.4 If `pendel.local` does not resolve

`.local` names are found by multicast DNS (mDNS), not by a DNS server. If `ssh` says *"Could not resolve hostname
pendel.local"*:

1. Use **PowerShell**, not Git Bash.
2. Check that the rig is powered, the Pi's Ethernet LEDs blink, and a minute has passed since power-on.
3. Try `ping pendel.local`. Windows 10 and 11 resolve `.local` names themselves on most networks. Apple's Bonjour
   service for Windows is a common add-on where they don't.
4. Use the **IP address** instead.
   - The Pi has had 192.168.0.100, .101 and .102 on the lab network (DHCP, so it can change; `CLAUDE.md`).
   - On the rig's own screen, a terminal command `hostname -I` prints it.
   - Or look at the router's list of DHCP clients.
   - Then: `ssh pendel@<ip address>`. For remote mode: `py\remote_ctrl.py --host <ip address>`.

> ⚠️ TODO / unverified: `py/fetch_runs.py` has the host fixed as `pendel@pendel.local` (`py/fetch_runs.py:14`) and
> no option to change it. Without `.local`, copy logs with `scp` (§6.5).

**"REMOTE HOST IDENTIFICATION HAS CHANGED".** SSH refuses to connect because the Pi's key differs from the one your
PC remembers. That is expected once if your PC knew the Pi that was replaced on 2026-10-23. Otherwise, stop:
another machine may be answering to that name. Only after you have checked the new fingerprint on the Pi itself
(§6.2), remove the old entry and log in again:

```powershell
# PC (PowerShell)
ssh-keygen -R pendel.local
```

### 6.5 Copying files

```powershell
# PC (PowerShell, in the repo folder)
scp pendel@pendel.local:Pendel-main/logs/<run folder>/meta.json .
scp -r pendel@pendel.local:Pendel-main/logs/<run folder> logs\
```

The first copies one file into the current folder; the second copies a whole run folder into `logs\`. For logs,
`py/fetch_runs.py` (§7.6) does the second for you. **Code does not travel by `scp`.** It goes through git (§6.6);
a file copied into `~/Pendel-main` would make the Pi's repository refuse the next push.

### 6.6 Getting the code onto the Pi

The Pi's `~/Pendel-main` is a git repository on branch `control-dev`. It accepts pushes straight into its working
files (`receive.denyCurrentBranch=updateInstead`, set up by `setup_pi.sh`). The routine, from `CLAUDE.md` and
`docs/working_with_the_pi.md`:

```powershell
# PC (PowerShell, in the repo folder): once
git remote add pi pendel@pendel.local:Pendel-main
```

```powershell
# PC (PowerShell, in the repo folder): every time the Pi should get your committed code
git push pi control-dev
ssh pendel@pendel.local "cd ~/Pendel-main && git log --oneline -1"
ssh pendel@pendel.local "cd ~/Pendel-main && make && make test"
```

- The second line confirms what is deployed. The third builds everything and runs the unit tests; nothing moves.
- `make` builds `build/main`, `build/pendctl` and `build/unit_tests`, deleting each old binary first, so a failed
  build never leaves a stale program to run.
- `make test` ends with `95/95 tests passed` at `fad2469`.

**Rules:**

- **Never edit files on the Pi.** The next push would be refused, and nobody would know what changed.
- **Never `git push --force`.**
- Push to GitHub only finished, tested work: it is the shared copy.
- Pushing to `pi` changes what the rig runs: run `make test` there before the next run on the rig.

**A freshly imaged Pi.** `setup_pi.sh`, run on that Pi:

- installs `g++ make git libgpiod-dev gpiod`;
- adds `gpio=24=op,dh` to `/boot/firmware/config.txt`, which holds the driver off from boot;
- creates the repository.

`bash setup_pi.sh --check` only reports. It needs Raspberry Pi OS **bookworm, 32-bit (armhf)**: the code uses the
libgpiod v1 API, which newer releases have dropped. `--rig` is for a Pi that replaces the rig's Pi under the same
name.

---

## 7. Running the local system (controller on the Pi)

### 7.1 What is always safe, and what needs the checklist

| Always safe (nothing moves) | Moves the cart or opens the real GPIO chip: pre-flight checklist first (§7.2) |
|---|---|
| `make`, `make test`, reading files and logs, `git log`, fetching logs, `build/pendctl --mode log-selftest` | `./demo.sh`, `./remote.sh`, `./swingup.sh`, `./pcbtest.sh`, `./start.sh`, `build/main`; `build/pendctl --mode move`, `damp`, `balance`, `shadow`, `encoder-log` on the real chip |

`build/pendctl --mode remote-sim` opens no GPIO at all and moves nothing (reference §5.2). It is the latency
measurement of §8.4.

### 7.2 Pre-flight checklist (every motion run)

From `demo.sh` and `CLAUDE.md`:

1. **Everyone near the rig knows what will move.**
   - Homing: the cart drives to each switch at 40 mm/s, then to the centre, about 25 s.
   - Balancing: up to 2 m/s² and 0.3 m/s, within ±150 mm of the centre.
2. **Rocker switch within reach.** Ctrl-C is a controlled stop.
3. **Cart roughly in the middle, not on a switch.** Push it by hand only while the driver is off (§7.5).
4. **Pendulum hanging and still.** Nothing on the track; hands away from the belt and pulleys.
5. **A spare 1 A fuse** for the motor supply (VMOT) at hand.
6. Nobody measuring on the Pi's header.

### 7.3 Start a balance session

Open an SSH session you can type into (`-t` keeps the keyboard connected to the program), then start the demo
script:

```powershell
# PC (PowerShell)
ssh -t pendel@pendel.local
```

```bash
# Pi (bash, in that SSH session)
cd ~/Pendel-main
./demo.sh
```

`demo.sh` prints its checklist and waits. Type `yes` and Enter. It then builds `build/pendctl` (and stops if that
fails) and starts `sudo build/pendctl --mode balance` on the real chip.

**What you see and do** (messages from `src/app/balance_mode.cpp`; numbers are examples):

1. **Header.**

   ```text
   balance session on gpiochip4 (pinctrl-bcm2711). PID Kp 62.78 Ki 97.51 Kd 6.492, a_max 2.0 m/s^2, v_max 0.30 m/s, soft limits -150/+150 mm about the track centre, engage |theta| < 4 deg, abort > 22 deg.
   Enter = next step (home / stop the run / re-centre), q + Enter or Ctrl-C = stop and end.
   calib: give the HANGING pendulum a ~20 deg swing and let go (pivot friction: 3-8 deg dies too fast). Hands off until 'calib: accepted' (~5 s). Driver OFF.
   ```

2. **Calibration (driver off).** Give the hanging pendulum a swing of about 20° and take your hands away. A status
   line appears every second; within about 5–10 s you see `calib: accepted c_bottom = … counts …`. The program has
   now found "straight down", and therefore "straight up" (reference §4.1). Stop the swing by hand and let the
   pendulum hang still.
3. **Homing.** Press Enter.
   - `home: driver ON, seeking the RIGHT switch`, then the left switch.
   - Then a line with the measured span, which must end in **`step mode OK`**.
   - Then `home: centred at pos …. Arms by itself when the pendulum hangs still.`
4. **Arming.** When the pendulum hangs still, `ARMED (run 1): …` appears.
5. **Catch.** Lift the pendulum to upright and hold the rod lightly near the top with your fingertips, hand still.
   - Within 4° of upright and still for 50 ms, the terminal beeps: `ENGAGED (theta …): let go in about 1 s`.
   - The cart moves 1–2 cm under your hand.
   - About 1 s after the beep, **let go cleanly**: open your fingers, no push.
6. **Balancing.**
   - A status line every 0.5 s: `theta … deg  x … mm  v … m/s  a … m/s^2`.
   - The motor hums faintly: the expected ±1–2-count limit cycle.
   - The outer cart loop slowly brings the cart back toward the centre.
7. **End of a run.** On the soft limit, a fall beyond 22°, the 60 s limit, or when you press Enter, it prints
   `STOPPING: <reason>` and brakes. Then a summary line: `run 1: <reason> after … s balanced. RMS theta …`.
   **Catch the pendulum** if it is falling, and let it hang still.
8. **Next run.** Press Enter to re-centre: the cart touches the nearer switch (the step-loss audit:
   `re-centre: … switch found +0.3 mm from where the step count puts it`), drives to the centre and re-arms. Repeat
   from step 5.
9. **End the session.** Type `q` and Enter. The program stops the cart, switches the driver off and writes the logs:

   ```text
   session finished: driver off.
   balance: FINISHED; 3 runs
     -> logs/2026-11-18T10-15-00_balance
   driver enable pin after the run (op dh = driver off):
   ```

   followed by the output of `pinctrl get 24`, which must contain **`op dh`**.

**Messages you may see** (more in `docs/stage1_demo.md`, "Messages"):

| Message | Meaning | What to do |
|---|---|---|
| `waiting to arm: … not hanging` / `… moving` | the arm check: within 10° of hanging, below 0.3 rad/s, cart still | let the pendulum settle, or stop it by hand |
| `STOPPING: soft limit within braking distance` | normal end: the cart reached its ±150 mm limit, including the braking distance | catch the pendulum; Enter to re-centre |
| `STOPPING: angle beyond theta_abort` | the pendulum fell (more than 22°) | catch it; Enter |
| `FAILED: track span …` | the switches were not where 1/8 step expects them | stop; see §10 |
| `FAILED: … switch found … mm from where the step count puts it` | step loss | stop; see §10 |
| `FAILED: calibration …` | no accepted swing within 120 s; nothing moved | restart, swing ~20°, hands off |

### 7.4 How balancing starts

There is no automatic swing-up in `./demo.sh`: **you lift the pendulum to upright by hand** (step 5), and the
supervisor engages the controller once it is near upright and still. The swing-up variant is `./swingup.sh`
(§7.7), where the cart pumps the pendulum up by itself.

### 7.5 Stopping safely

| Action | Effect |
|---|---|
| Enter during a run | ends that run with a controlled stop; the cart is then held in place |
| `q` + Enter, or Ctrl-C | controlled stop, driver off, logs written, program ends |
| Rocker switch | hard stop: motor and Pi lose power at once; that session's logs are lost |

After any unexpected end, check the driver-enable pin:

```bash
# Pi (bash)
pinctrl get 24
```

It must show `op dh` (output, driven high = driver off). If it does not, switch the driver off by hand:

```bash
# Pi (bash)
pinctrl set 24 op dh
```

### 7.6 Where the logs go, and how to look at them

Each session writes `~/Pendel-main/logs/<date>T<time>_balance/` on the Pi, containing:

- `data.csv`: every 2 ms tick;
- `edges.csv`: every encoder edge;
- `meta.json`: build, results, per-run summary;
- `config_used.conf`, `config_source.conf`: the configuration the run used.

Reference §5.8 describes the columns. Copy and analyse them on the PC:

```powershell
# PC (PowerShell, in the repo folder)
.venv\Scripts\python py\fetch_runs.py --last 1
.venv\Scripts\python py\stage1_report.py balance
.venv\Scripts\python py\plot_run.py
```

- `fetch_runs.py` copies the newest run into the repo's `logs\` folder (`--last 3` for three).
- `stage1_report.py balance` prints a table per run (duration, how it ended, RMS angle, saturation, drift fit with a
  suggested `encoder.trim` change) and saves `balance_runs.png` in the run folder.
- `plot_run.py` plots the run's columns into `plot.png`.

`logs/` is not committed to git (`.gitignore`).

### 7.7 Variants

- **One-run settings:** anything after the script name is passed on to `pendctl`, e.g.
  `./demo.sh --set encoder.trim=0.3` (§9).
- **Swing-up:** `./swingup.sh`, with its own checklist. After homing, wait for `READY`, then Enter starts a run:
  - the cart kicks and pumps for ~15–25 s, swinging ±60–100 mm;
  - it hands over to the balance controller near the top;
  - at the end it pumps the energy back out.

  During a run, Enter brings it down gently; `q` or Ctrl-C is a hard stop (the cart stops, but the pendulum keeps
  swinging). **Stay outside the pendulum's circle**, radius about 0.5 m around the pivot.
- **Sensors-only check (shadow mode):** the full controller runs on the real sensors, but the motor lines are never
  requested, so nothing can move. Use it to check signs after any mechanical change (`docs/stage1_demo.md`, "After a
  mechanical rebuild"). It opens the real chip, read only:

  ```bash
  # Pi (bash, in an ssh -t session)
  cd ~/Pendel-main
  make -s build/pendctl && sudo build/pendctl --mode shadow --set shadow.chip=gpiochip4 --set shadow.allow_real_chip=true
  ```

  Lean the upright pendulum's top toward +x (the LEFT switch): `a_cmd` must be positive. Ctrl-C ends it.
  `&&` makes sure a failed build never runs an old binary.

---

## 8. Running the remote system (controller on the PC)

In remote mode the Pi does everything as in §7 except the controller itself. That runs on your PC in
`py/remote_ctrl.py`, and the Pi applies each command exactly $N$ = 4 ticks (8 ms) after the sample it was computed
from. Why, and what that does to stability: reference §3.7. The protocol: reference §5.6.

### 8.1 Requirements

- **Wired Ethernet for the PC.** Wi-Fi measured worse and can drop out entirely, which no delay covers.
- **The same code version on both machines.** The Pi's build and your `py/remote_ctrl.py` must speak the same
  protocol version (4 at `fad2469`). Compare `git log --oneline -1` on the PC and on the Pi. With an older copy of
  `remote_ctrl.py`, the link never comes up.
- **Python 3** on the PC. `remote_ctrl.py` uses only the standard library, so the venv works, and so does plain
  `py`.
- **Network.**
  - The Pi listens on **UDP port 9760** (`remote.port`). The PC sends from a port Windows picks and receives the
    replies on it.
  - If Windows Firewall asks about Python, **allow it on private networks**. Replies to the PC's own HELLO normally
    pass without a rule (`docs/remote.md`).
  - The lab network should be set as a *Private* network on your PC: Settings → Network & internet → Ethernet →
    Network profile type.

> ⚠️ TODO / unverified: the repo documents nothing beyond "allow private networks" for the firewall, and nothing
> about a firewall on the Pi. If the link never comes up although both programs run, the firewall is the first
> suspect.

### 8.2 Start order: PC first, then the Pi

**1. On the PC**, in its own PowerShell window:

```powershell
# PC (PowerShell, in the repo folder): window 1
.venv\Scripts\python py\remote_ctrl.py --host pendel.local
```

Expected:

```text
remote_ctrl: PC controller for pendel.local 192.168.0.101:9760 (high priority class, time-critical thread, 1 ms timer)
remote_ctrl: waiting for the Pi (./remote.sh, or build/pendctl --mode remote-sim). Ctrl-C ends.
```

By default the program raises its own priority (for this process only). On a test laptop that cut its worst answer
time from 15.9 ms to 0.99 ms. Leave it that way; `--normal-priority` switches it off.

**2. On the Pi**, from a second window:

```powershell
# PC (PowerShell): window 2
ssh -t pendel@pendel.local
```

```bash
# Pi (bash, in that SSH session)
cd ~/Pendel-main
./remote.sh
```

`remote.sh` has the checklist of §7.2 plus a fifth point: the PC program is running and shows "waiting for the Pi".
Type `yes`. The session then runs exactly as in §7.3 (calibration, Enter to home, lift at ARMED, let go after the
beep), with one more header line:

```text
REMOTE MODE: the controller runs on the PC (py/remote_ctrl.py, UDP port 9760). Constant delay 4 ticks = 8.0 ms; a run aborts after 5 missing commands in a row or 100 ms without replies; arming waits for the PC. Everything else (limits, stops, homing, keys) stays on the Pi.
```

### 8.3 Confirming the link works

| Where | What you should see |
|---|---|
| PC | `remote_ctrl: Pi 192.168.0.101 params: Kp 62.78 Ki 97.51 Kd 6.492, a_max 2.0 m/s^2, v_max 0.30 m/s, cart loop kx 0.41 kv 1.00 ki 0.050; constant delay 4 ticks = 8.0 ms`: the PC received the Pi's gains |
| Pi | `remote: link UP, PC <PC address>:<port>` |
| Pi, every 5 s while not balancing | `remote: link up, PC …; last 5 s: 2500 replies, worst round trip 1.20 ms (delay 8.0 ms), 0 late` |
| Pi, while balancing | the status line ends in `\| PC: worst … ms, … late, … missed` |
| PC, every 1–5 s | `Pi BALANCE/ENGAGED  theta …  \| samples …, lost 0, PC processing max … us` |
| Pi, at the end | `remote: worst round trip … ms (delay 8.0 ms), … late replies, … missed ticks, max \|PC - local PID\| 0 m/s^2` |

The last number compares the PC's commands with the Pi's own shadow copy of the controller. It must be **0**: any
other value means the two sides do not compute the same controller.

"Worst round trip" must stay below the delay (8.0 ms). Late replies and missed ticks should be 0, or a rare few.

### 8.4 Measuring the latency (no motion)

To choose the delay $N$ for your PC and network, run the remote loop against a **simulated** pendulum on the Pi:
no GPIO is opened and nothing moves. This is the procedure in `docs/remote.md`, "Procedure (wired PC)".

```powershell
# PC (PowerShell, in the repo folder): window 1
.venv\Scripts\python py\remote_ctrl.py --host pendel.local
```

```bash
# Pi (bash, in an ssh -t session)
cd ~/Pendel-main
make build/pendctl && sudo build/pendctl --mode remote-sim --set remote.delay_ticks=10 --set remote.sim_seconds=600 --set plant.coulomb=0
```

- `remote.delay_ticks=10` gives plenty of room while measuring.
- `remote.sim_seconds=600` balances the simulated pendulum for 10 minutes.
- `plant.coulomb=0` is needed with the current pivot friction; without it the simulated rod sticks at upright and
  the cart runs off (`docs/remote.md`, "Procedure (wired PC)").
- `sudo` gives the loop its real-time priority, as on the rig.

A 10-minute run is lost if the SSH session drops, so keep the PC awake. `docs/remote.md` suggests `tmux` or `nohup`.

> ⚠️ TODO / unverified: whether `tmux` is installed on the rig's Pi is not recorded (`setup_pi.sh` does not install
> it).

Then fetch the run and let the report choose $N$:

```powershell
# PC (PowerShell, in the repo folder)
.venv\Scripts\python py\fetch_runs.py --last 1
.venv\Scripts\python py\remote_report.py logs\<time>_remote-sim
```

The report prints the round-trip distribution, how many replies would have been late for each $N$, and the
smallest $N$ that covered them all. It saves `latency.png` and, if it finds the PC program's log next to the run,
splits each round trip by direction. Rule (`docs/remote.md`): the smallest $N$ with worst round trip
< $N$·2 ms − 0.1 ms, plus one tick of margin. Then run once more with that $N$; it must finish with 0 missed ticks.
To make it the default, set `remote.delay_ticks` in `config/pendctl.conf` (§9).

`py/net_probe_pi.py` and `py/net_probe_pc.py` measure the bare network (UDP port 9750) without `pendctl`; their
docstrings show the commands.

### 8.5 Stopping safely

The keys on the Pi are the same as in §7.5: Enter ends a run, `q` + Enter or Ctrl-C ends the session, the rocker
switch is the hard stop. Stop the Pi side first. The PC program then ends by itself
(`remote_ctrl: the Pi ended the session`) and writes `logs\<time>_remote-pc\samples.csv` and `summary.json` on the
PC. Ctrl-C ends it at any time.

### 8.6 What happens if the link drops

Everything that keeps the rig safe runs on the Pi (reference §4.6):

- **Missing replies.** A missing reply makes that tick repeat the previous command. 5 missing in a row
  (`remote.max_miss`), or no valid reply for 0.1 s (`remote.link_timeout`), abort the run with a controlled stop on
  the Pi: `STOPPING: remote link (commands missing or link down)`.
- **No arming while the link is down.** The Pi shows
  `waiting to arm: remote link (commands missing or link down)` until the PC answers again.
- **In remote swing-up** (`./swingup.sh --remote`): a lost link while pumping removes the energy; during the balance
  phase the cart brakes and then removes the energy.
- **If you stop the PC program in the middle of a run,** that is exactly this case: the Pi brakes. It is safe, but
  stop the Pi side first when you can.

---

## 9. Changing gains and settings

### 9.1 Where settings live

All settings are in **`config/pendctl.conf`**. Each line is `key = value  # unit and reason`, and the comments
explain where each value came from. Every run saves the values it actually used in its `config_used.conf`.

**The PID gains are not in the file.** At every start the program computes them by pole placement (reference §3.2)
from the identified pendulum (`plant.w0`, `plant.bJ`) and three design ratios. To change the controller, change
those:

| Key | Now | Effect of raising it | Watch out |
|---|---|---|---|
| `ctrl.wc_over_w0` | 2.0 | faster, stiffer angle loop | less delay margin and more command noise (3.0 gave a 30 ms delay margin vs 48 ms on the old pivot, `docs/gateB_report.md`) |
| `ctrl.zeta` | 0.7 | more damping, less overshoot | larger $K_d$, so more noise; 0.6–0.7 minimised the peak sensitivity |
| `ctrl.p_over_w0` | 0.5 | faster integral action | 0 = PD: the cart then keeps more velocity after a catch |
| `ctrl.est_fc` | 14 Hz | velocity filter cut-off: less lag | more noise; at 20 Hz a ~28 Hz rattle grew in 3 of 17 rig runs (0 of 7 at 14 Hz) |
| `balance.a_max`, `balance.v_max` | 2.0 m/s², 0.3 m/s | more authority | step loss, the 1 A VMOT fuse; the typo guards `move.max_accel` = 2.0 and `move.max_speed` = 0.3 refuse higher values |
| `ctrl.cart_kx`, `ctrl.cart_kv`, `ctrl.cart_ki` | 0.41, 1.0, 0.05 | firmer cart centring | a slowly growing 0.1–0.3 Hz sway means too fast; all three 0 = the Stage 1 angle-only controller |
| `encoder.trim` | 0 counts | moves the upright reference | change it only from the drift that `stage1_report.py balance` reports |
| `remote.delay_ticks` | 4 | covers longer network stalls | each tick costs ~2.1° of phase margin (reference §3.7) |

### 9.2 Trying a value for one run

Pass it after the script name. The scripts forward everything to `pendctl`, and it overrides the file for that run
only:

```bash
# Pi (bash, in an ssh -t session)
cd ~/Pendel-main
./demo.sh --set ctrl.est_fc=12
```

Several `--set key=value` can follow each other. The run's `config_used.conf` marks overridden values with
`(override)`.

### 9.3 Changing a value permanently

Edit `config/pendctl.conf` **on the PC**, then commit and push. Never edit it on the Pi.

```powershell
# PC (PowerShell, in the repo folder)
git pull
git diff
git add config/pendctl.conf
git commit -m "config: <what changed and why>"
git push pi control-dev
```

`git diff` shows your change before you commit it.

### 9.4 Doing it safely

- **One change at a time**, and record why in the config comment, the way the existing comments do.
- **Check the margins before a gain change reaches the rig.** The linear model in `py/pendel/linear.py`
  (`margins()`) gives phase, gain and delay margins for any design (reference §3.7 shows its output).
  - The design scripts `py/gateb_design.py` and `py/zeta_study.py` have the **old pivot's plant hard-coded**
    (ω₀ 4.931, b/J 0.024); edit them or call `pendel.linear` directly.
  - Running `py/gateb_design.py` or `py/zeta_study.py` also **overwrites figures in `docs/img/`**, which are
    committed.
- **Validation is not stability.** The program refuses obviously wrong values at start: $K_p \le 0$, limits above
  the typo guards, soft limits that don't fit inside the switches. It does not check that a design is stable.
- **Never flip `calib.theta_sign` or `calib.x_sign`** without a shadow-mode check (§7.7). A wrong sign turns the
  controller into positive feedback; the supervisor aborts within about 0.5 s, but the cart jumps first.
- **First runs of a new setting:** reduced limits, one catch at a time, `q` in between.

---

## 10. Troubleshooting

**Git and GitHub**

| Symptom | Likely cause | Fix |
|---|---|---|
| `git clone` keeps asking for a user name, or says `Repository not found` | the repo is private and your account has no access, or git used another account | make sure your GitHub account has access; sign in through the browser window (§4.3 A) or use SSH (§4.3 C) |
| `fatal: invalid reference: control-dev` | GitHub has no `control-dev` branch | clone the Pi's copy instead (TODO in §4.2) |
| `Permission denied (publickey)` from `git@github.com` | your SSH key is not added to GitHub | add the `.pub` key on GitHub; test with `ssh -T git@github.com` |
| `git push pi` rejected: `fetch first` / `non-fast-forward` | someone pushed to the Pi since your last pull | `git pull --rebase pi control-dev`, then push again |
| `git push pi` rejected: `Working directory has unstaged changes` | someone edited files on the Pi | stop; look at `git status` on the Pi before anything else; never force |

**Python**

| Symptom | Likely cause | Fix |
|---|---|---|
| `python` is not recognised, or opens the Microsoft Store | Python not installed or not on PATH; the Store placeholder alias | install as in §2; open a new window; use `py -3.14`; check "Manage app execution aliases" |
| `py` is not recognised | classic installer without the launcher, or not on PATH | install the Python install manager (§2) |
| `Activate.ps1 cannot be loaded because running scripts is disabled` | PowerShell execution policy | `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`, or skip activation and call `.venv\Scripts\python` |
| `ModuleNotFoundError: No module named 'numpy'` | the venv's Python was not used, or the packages are not installed | run scripts with `.venv\Scripts\python`; reinstall with `.venv\Scripts\python -m pip install -r py\requirements.txt` |
| `pip` cannot find `numpy==2.5.3` or `scipy==1.18.1` | Python older than 3.12 | install 3.14, delete `.venv`, create it again (§5) |

**Connecting to the Pi**

| Symptom | Likely cause | Fix |
|---|---|---|
| `Could not resolve hostname pendel.local` | Git Bash instead of PowerShell; Pi off or still booting; cable | use PowerShell; check power and the Ethernet LEDs; use the IP address (§6.4) |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | the Pi was replaced (2026-10-23), or a different machine answers | check the new fingerprint on the Pi itself (§6.2), then `ssh-keygen -R pendel.local` (§6.4) |
| `Permission denied (publickey,password)` | wrong password, or the key step was not done | repeat §6.3; check the password |
| `py/fetch_runs.py` fails with `Host key verification failed` or `Permission denied` | it runs ssh with `BatchMode`: no prompts, so it needs a key and a known host | log in once by hand (§6.2) and install your key (§6.3) |
| `make` fails on the Pi | the pushed code does not compile | fix on the PC, commit, push again; never run an old build |

**Starting `pendctl`**

| Symptom | Likely cause | Fix |
|---|---|---|
| `pendctl: refusing gpiochip4 ('pinctrl-bcm2711'): the real rig needs allow_real_chip=true` | `pendctl` started without the scripts' settings; this refusal is the safety default | use the script (`./demo.sh` …), or add `--set <mode>.chip=gpiochip4 --set <mode>.allow_real_chip=true` on purpose |
| `pendctl: cannot open gpiochip2` | the default chip is the simulated `gpio-mockup` chip, which is not loaded | you meant the rig: use the scripts. For mock tests, load the module first (`docs/stage0_3_safe_bringup.md`) |
| `balance: Axis step thread: SCHED_FIFO failed (run with sudo)` (also `move:`, `damp:`), or warnings like `warning: mlockall: …` | started without `sudo`. The motor modes refuse to start; the others run without real-time priority | use the scripts, which use `sudo`; for other modes put `sudo` in front |
| `cannot request encoder lines: Device or resource busy` (or the German `Fehler: konnte Events nicht anfordern`) | another program still holds the GPIO lines, e.g. an earlier `pendctl` or `main` | find it with `pgrep -a pendctl` and `pgrep -a main`, end it, run `pinctrl get 24` |
| `balance: a limit switch is pressed at start; refusing` | the cart is on a switch | push the cart to the middle by hand (driver off) |

**Encoder and calibration**

| Symptom | Likely cause | Fix |
|---|---|---|
| The calibration says `too few edges` and the count stays at 0 | no swing; encoder not plugged in (J4) or not powered | give a ~20° swing; with the power **off**, check the J4 connector |
| `calib: … not a free swing (hand contact?)`, or never `accepted` | a hand on the rod, a swing too small (dies in the window) or too large | hands off, ~20° swing. If the pivot was changed, the plant needs re-identifying (`plant.*`, `docs/stage1_demo.md` "After a mechanical rebuild") |
| A run aborts with `encoder fault` | lost or impossible edges: very fast motion, or the pendulum taken off and put back while running | restart (the reference is re-fitted at every start); check `encoder_same_type` and `encoder_full_batches` in `meta.json`; if it repeats, check the encoder wiring (J4) with the power off |
| Upright drifts between sessions | normal: no index pulse; the start-up fit scatters by about ±0.5 counts | nothing to fix; trim only from consistent drift reports |

**Motor and cart**

| Symptom | Likely cause | Fix |
|---|---|---|
| The cart does not move; homing ends with `switch not found before the give-up position` | no motor power: fuse F2 blown, VMOT off, motor unplugged. The step count runs while the cart stands still | `q`; power off; check F2 and the J9 plug. Measure VMOT only at C5's + leg, never on the header |
| `FAILED: track span …` | the driver is not in 1/8 step (MS1/MS2), or the track changed | stop; check the MS1/MS2 wiring. If the track really changed, set `balance.span_usteps` to the measured span (§9) |
| `FAILED: … switch found … mm from where the step count puts it` | step loss during the last run | lower `balance.a_max`/`v_max` for the next runs; check the belt |
| The motor stays stiff and hums after a crash | the driver was left enabled | `pinctrl get 24`; if not `op dh`: `pinctrl set 24 op dh` |

**Control behaviour**

| Symptom | Likely cause | Fix |
|---|---|---|
| The pendulum is thrown over right after ENGAGED, within ~0.5 s | a sign error after a mechanical change (θ or $x$ direction) | stop; shadow-mode check (§7.7); fix `calib.theta_sign` / `calib.x_sign` only after that check |
| Loud buzzing or rattling at ~28 Hz, command saturating | a drive-side mechanical mode pumped by the D term (`docs/stage1_demo.md`) | Enter to stop the run. Lowering `ctrl.est_fc` (e.g. `--set ctrl.est_fc=12`) reduces the loop gain there |
| A ~2 Hz oscillation right after the beep, while you still hold the rod | your hand is fighting the controller | let go cleanly about 1 s after the beep |
| Oscillation grows after a gain increase | too little phase margin | go back to the previous values; reduce `ctrl.wc_over_w0`; check margins first (§9.4) |
| A slow sway, 0.1–0.3 Hz, that grows | the outer cart loop is too fast for the rig | halve `ctrl.cart_kx` and `ctrl.cart_kv` (`docs/stage3_cascade.md`) |
| The cart drifts steadily to one side | an upright reference error δ: each count is 25.7 mm/s² of drift | run `py/stage1_report.py balance`; apply its suggested `encoder.trim` |
| Runs end at the soft limit right after a swing-up hand-over | hand-over at high cart speed | expected now and then; `swingup.v_sw` gates it (`docs/stage3_cascade.md`, "Stage 3b") |

**Remote mode**

| Symptom | Likely cause | Fix |
|---|---|---|
| The PC stays at `waiting for the Pi` | Pi side not started; wrong `--host`; firewall; **different protocol versions** (code on PC and Pi differ) | start `./remote.sh` or `remote-sim`; try `--host <ip address>`; allow Python on private networks; compare `git log --oneline -1` on both |
| The Pi shows `waiting to arm: remote link (commands missing or link down)` | the PC program is not running or its packets are blocked | start `py\remote_ctrl.py` first; check the firewall |
| Runs abort with `remote link`; many late replies | round-trip stalls longer than the delay: Wi-Fi, power saving, a busy PC, `--normal-priority` | wired Ethernet; close heavy programs; measure (§8.4); raise `remote.delay_ticks` to cover the measured worst case |
| `remote-sim` runs end after a few seconds, the simulated cart running away | the simulated pivot friction makes the rod stick at upright | add `--set plant.coulomb=0` (§8.4) |
| `max \|PC - local PID\|` is not 0 | the PC and the Pi do not compute the same controller | different code versions; update both and rebuild on the Pi |

**The Pi itself**

| Symptom | Likely cause | Fix |
|---|---|---|
| Reboots, or `vcgencmd get_throttled` is not `throttled=0x0` | undervoltage or overheating | find the cause (supply, cooling) before any motion run |

---

## 11. Quick-reference cheat sheet

**One-time setup (PC):**

```powershell
# PC (PowerShell)
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
git config --global core.sshCommand "C:/Windows/System32/OpenSSH/ssh.exe"
py install 3.14
cd $HOME\Documents
git clone https://github.com/simoncorazza7/Pendulum.git Pendel-main
cd Pendel-main
git switch control-dev
py -3.14 -m venv .venv
.venv\Scripts\python -m pip install -r py\requirements.txt
ssh-keygen -t ed25519 -C "your-name"
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh pendel@pendel.local "mkdir -p ~/.ssh && chmod 700 ~/.ssh && tr -d '\r' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
git remote add pi pendel@pendel.local:Pendel-main
```

(`git switch control-dev`: see the TODO in §4.2.)

**Update the code and the Pi:**

```powershell
# PC (PowerShell, in the repo folder)
git pull
git push pi control-dev
ssh pendel@pendel.local "cd ~/Pendel-main && make && make test"
```

**Local mode** (after the checklist in §7.2):

```powershell
# PC (PowerShell)
ssh -t pendel@pendel.local
```

```bash
# Pi (bash)
cd ~/Pendel-main
./demo.sh
```

Then: type `yes`, swing ~20°, Enter (home), lift at ARMED, let go ~1 s after the beep, Enter (re-centre), and
finally `q` + Enter. Afterwards `pinctrl get 24` must show `op dh`.

**Remote mode** (PC first):

```powershell
# PC (PowerShell, in the repo folder): window 1
.venv\Scripts\python py\remote_ctrl.py --host pendel.local
```

```bash
# Pi (bash, in ssh -t from window 2)
cd ~/Pendel-main
./remote.sh
```

**Latency measurement** (no motion):

```bash
# Pi (bash), with py\remote_ctrl.py already running on the PC
cd ~/Pendel-main
make build/pendctl && sudo build/pendctl --mode remote-sim --set remote.delay_ticks=10 --set remote.sim_seconds=600 --set plant.coulomb=0
```

**After a session (PC):**

```powershell
# PC (PowerShell, in the repo folder)
.venv\Scripts\python py\fetch_runs.py --last 1
.venv\Scripts\python py\stage1_report.py balance
.venv\Scripts\python py\remote_report.py logs\<time>_remote-sim
```

**Stopping:** Enter = end the run · `q` + Enter or Ctrl-C = controlled stop, driver off · rocker = hard stop ·
then `pinctrl get 24` → `op dh` (else `pinctrl set 24 op dh`).

---

Sources for the installation steps: the Python install manager and the classic installer, <https://docs.python.org/3/using/windows.html>;
Git for Windows, <https://git-scm.com/downloads/win>. Everything about the rig comes from the repo (`CLAUDE.md`,
`docs/working_with_the_pi.md`, `docs/stage1_demo.md`, `docs/remote.md`, the scripts and the code).
