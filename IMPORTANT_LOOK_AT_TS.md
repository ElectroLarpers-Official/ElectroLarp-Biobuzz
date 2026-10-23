# IMPORTANT, LOOK AT THIS

We now check commit messages automatically. Every commit has to look like this:

```
D11 Regular Meet: what you changed
```

Bad messages get refused when you commit, and bad pushes get a red X on GitHub (and you get an email). Do the setup below **once** on your Windows laptop.

---

## Setup (once, Windows)

Do everything in **Android Studio's Terminal** tab (bottom of the window, or View → Tool Windows → Terminal). Run the commands one at a time.

### 1. Pull the latest code
Save your uncommitted changes for a sec:
```powershell
git stash -u
```
Pull:
```powershell
git pull --rebase
```
Bring your changes back:
```powershell
git stash pop
```
If `git pull --rebase` shows conflicts, run `git rebase --abort` and ask whoever manages the repo before doing anything else.

### 2. Fix the hook file for Windows
Windows can save the hook with the wrong line endings, so delete it and get it again:
```powershell
Remove-Item .githooks\commit-msg
```
```powershell
git checkout -- .githooks/commit-msg
```
Check it, the output should contain `w/lf`:
```powershell
git ls-files --eol .githooks/commit-msg
```

### 3. Sync Gradle
In Android Studio click **File → Sync Project with Gradle Files** (or just build/deploy like normal). This turns on the commit check and makes `git pull` safe, by itself.

### 4. Check it worked
Should print `.githooks`:
```powershell
git config core.hooksPath
```
Should print `true`:
```powershell
git config pull.rebase
```
Should say `Bad commit message: ...` (nothing actually gets committed):
```powershell
git commit --allow-empty -m "fixed stuff"
```

If the first two printed nothing, run these and check again:
```powershell
git config core.hooksPath .githooks
```
```powershell
git config pull.rebase true
```
```powershell
git config rebase.autoStash true
```

Done. You never have to do this again on this laptop.

---

## Commit message format

`D[day] [Regular or Home] Meet: [what changed]`

- **Day**: look at the last commit's number (`git log --oneline -5`)
  - same meet as the last commit → **same number** (last was D11 → D11)
  - new meet → **last number + 1** (last was D11 → D12)
  - skipping numbers or going backwards gets refused
- **Regular** = normal team meeting, **Home** = working from home

Examples:
```
D11 Regular Meet: fix turret offset
D12 Home Meet: add close paths
```

## If your commit gets refused
Nothing was committed, just commit again with a fixed message:
```powershell
git commit -m "D11 Regular Meet: what you changed"
```

Already committed with a bad message but didnt push yet:
```powershell
git commit --amend -m "D11 Regular Meet: what you changed"
```

`git pull` refused with `merges an old copy of Biobuzz back in`:
```powershell
git merge --abort
```
```powershell
git pull --rebase
```

## Android Studio rules
- In the commit window, keep **Run Git hooks** checked (under the message box). Unchecking it skips the check and your push gets a red X.
- In **Git → Update Project**, pick **Rebase**, not Merge.
- Never use `--no-verify`.

## Errors
- `/usr/bin/env: 'bash\r': No such file or directory` → redo step 2.
- `cannot pull with rebase: You have unstaged changes` → redo step 4's fallback commands (the `rebase.autoStash` one fixes it), or `git stash`, `git pull`, `git stash pop`.

More git help: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/gitInstructions.md`

---

## Setup (once, Mac)

Same idea as Windows, but no line ending fix needed. Run in Android Studio's Terminal tab (or the Terminal app, inside the project folder), one at a time.

### 1. Pull the latest code
Save your uncommitted changes for a sec:
```bash
git stash -u
```
Pull:
```bash
git pull --rebase
```
Bring your changes back:
```bash
git stash pop
```
If `git pull --rebase` shows conflicts, run `git rebase --abort` and ask whoever manages the repo before doing anything else.

### 2. Sync Gradle
In Android Studio click **File → Sync Project with Gradle Files** (or just build/deploy like normal), or from the terminal:
```bash
./gradlew help -q
```

### 3. Check it worked
Should print `.githooks`:
```bash
git config core.hooksPath
```
Should print `true`:
```bash
git config pull.rebase
```
Should say `Bad commit message: ...` (nothing actually gets committed):
```bash
git commit --allow-empty -m "fixed stuff"
```

If the first two printed nothing, run these and check again:
```bash
git config core.hooksPath .githooks
```
```bash
git config pull.rebase true
```
```bash
git config rebase.autoStash true
```

If the commit check errors with `permission denied`, make the hook runnable:
```bash
chmod +x .githooks/commit-msg
```
