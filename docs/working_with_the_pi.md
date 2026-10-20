# Working with the Pi (for Simon's assistant)

How to reach the files on the rig's Pi, change them on your own computer, and send the changes to the Pi. The same
steps as a page with copy buttons: https://claude.ai/artifact/WHmkkUcREiFBZbSf3yD82k (Simon shares it).

The code on the Pi (`~/Pendel-main`, branch `control-dev`) is a git repository that accepts pushes straight into its
working files. You keep a copy on your computer, edit and commit there, and `git push` updates the Pi. **Never edit
files on the Pi itself:** the next push would be refused, and nobody would know what changed.

## Before anything: the rig moves

- **Needs Simon's go for that run:** anything that moves the cart or opens the real GPIO chip:
  - `./demo.sh`, `./swingup.sh`, `./remote.sh`, `./start.sh`, `build/main`;
  - `build/pendctl --mode move | damp | balance | shadow | encoder-log`.
- **Safe on your own:** `make`, `make test`, copying logs, reading files, `build/pendctl --mode log-selftest`.
- **When the rig is powered:** keep the motor power (rocker) switch within reach. Ctrl-C is a controlled stop.
- **After anything unexpected:** check that the driver is off. `pinctrl get 24` must show `op` `dh`;
  `pinctrl set 24 op dh` turns it off.

## Set up once (Windows, PowerShell; Git Bash can't resolve `pendel.local`)

1. **Install** Git for Windows and Python 3.11 or newer (add it to PATH). Then:
   ```
   git config --global core.sshCommand "C:/Windows/System32/OpenSSH/ssh.exe"
   git config --global user.name "Your Name"
   git config --global user.email "you@example.com"
   ```
2. **Make an SSH key:** `ssh-keygen -t ed25519 -C "your-name"` (Enter for the defaults).
3. **Put the key on the Pi.** Type the Pi's password, which Simon gives you, once:
   ```
   type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh pendel@pendel.local "mkdir -p ~/.ssh && chmod 700 ~/.ssh && tr -d '\r' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
   ```
   - Accept the fingerprint only if it is `SHA256:Wnp+aM+kwJnd9hLdTFa+kKDR0QoMufwu4zVaZaG2wlM` (ED25519).
   - Check: `ssh pendel@pendel.local hostname` prints `pendel` without asking for a password.
4. **Get the code:** this names the Pi `pi`, as in Simon's setup.
   ```
   cd $HOME\Documents
   git clone -o pi pendel@pendel.local:Pendel-main
   cd Pendel-main
   git switch control-dev
   ```
5. **Install the analysis tools:**
   `python -m venv .venv`, then `.venv\Scripts\python -m pip install -r py\requirements.txt`.

## Look at files and logs

- **Shell on the Pi:** `ssh pendel@pendel.local`, then `cd ~/Pendel-main`. Useful there:
  - `git log --oneline -5`: what is deployed;
  - `ls logs`: one folder per run;
  - `less config/pendctl.conf`: the settings, with units in the comments.
- **Copy runs to your computer:** `.venv\Scripts\python py\fetch_runs.py --last 3` (into `logs\`), then
  `.venv\Scripts\python py\plot_run.py`.

## Change something and send it to the Pi

1. `git pull pi control-dev`: start from the latest version.
2. Edit on your computer. To try a value for one run only, use an override instead of editing:
   `./demo.sh --set encoder.trim=-1.6`.
3. Check and commit: `git status`, `git diff`, `git add <file>`,
   `git commit -m "what changed and why"`. Keep commits small.
4. `git push pi control-dev`: the Pi's files now match your commit.
5. Build and test on the Pi; nothing moves: `ssh pendel@pendel.local "cd ~/Pendel-main && make && make test"`.
   The build deletes the old program first, so a failed build never leaves a stale one to run.
6. Confirm: `ssh pendel@pendel.local "cd ~/Pendel-main && git log --oneline -1"`.

## When something goes wrong

| You see | Meaning | Do |
|---|---|---|
| push rejected: `fetch first` / `non-fast-forward` | someone pushed since your pull | `git pull --rebase pi control-dev`, push again |
| push rejected: `Working directory has unstaged changes` | someone edited files on the Pi | stop, tell Simon; don't force |
| `Could not resolve hostname pendel.local` | Pi off or elsewhere, or you're in Git Bash | use PowerShell; check cable and power; ask Simon |
| `Permission denied (publickey)` | your key isn't on the Pi | repeat set-up step 3 |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | another machine answers to that name | stop, tell Simon |
| the build fails on the Pi | the pushed code doesn't compile | fix, commit, push again; never run an old build |

Never `git push --force`, and never push anywhere but `pi`.

## Running the rig: only together with Simon

`ssh -t pendel@pendel.local`, then `cd ~/Pendel-main && ./demo.sh` (or `./swingup.sh`, `./remote.sh`). Each starts
with a checklist. After every run: `pinctrl get 24` must show `op dh`.

**macOS / Linux:** use Terminal, skip the `core.sshCommand` line, put the key on the Pi with
`ssh-copy-id pendel@pendel.local`, and use `.venv/bin/python` with forward slashes.
