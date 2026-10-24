# Git Usage Notes

General git commands and commit message format for the team. Provides info to pull from Pedro repo and switch branches per season/comp

---

## Repo Setup (one time, per machine)

After cloning the repo, add Pedro's original repo as a second remote so you can pull their updates later. This only needs to be done once per machine:

```bash
git remote add upstream https://github.com/Pedro-Pathing/Quickstart.git
```

Check your remotes are set up right:
```bash
git remote -v
```
Should show `origin` (our repo) and `upstream` (Pedro's repo).

The commit message check and pull settings turn on **automatically** the first time Android Studio syncs or builds the project (`TeamCode/build.gradle` runs them), so normally you dont have to do anything. They are:
- `core.hooksPath .githooks` - git uses the hooks in our `.githooks` folder, so a bad commit message gets refused before the commit is even made
- `pull.rebase true` - `git pull` rebases instead of merging. A plain merge pull is what kept bringing old commits with bad messages back after they got fixed
- `rebase.autoStash true` - `git pull` stashes your uncommitted changes and brings them back by itself

Check they're on (should print `.githooks` and `true`):
```bash
git config core.hooksPath
git config pull.rebase
```

If not (git wasnt installed when you built, or you havent built yet), run them by hand:
```bash
git config core.hooksPath .githooks
git config pull.rebase true
git config rebase.autoStash true
```

In Android Studio's **Git → Update Project** dialog, pick **Rebase**, not Merge.

---

## Commit Message Format

Format: `D[day] [Regular or Home] Meet: [what changed]`

Example: `D10 Regular Meet: fix mirroring`

### Day
Meet counter, allows to easily navigate changes and roll back if needed. Look at the last commit's number:
- same meet as the last commit → **same number** (last was D10, you're still at that meet → D10)
- new meet → **last number + 1** (last was D10, this is the next meet → D11)

Skipping numbers or going backwards gets refused. Check the last number with:
```bash
git log --oneline -5
```

### Type of Meet
- **Regular** - normal team meeting
- **Home** - working from home

### Examples
```
D10 Regular Meet: fix mirroring
D10 Regular Meet: add 3D interpolator with tuned shot points
D11 Home Meet: add panels logging
```

### What gets checked
The hook (`.githooks/commit-msg`) checks every commit you make, and GitHub Actions (`.github/workflows/commit-messages.yml`) checks every push to `Biobuzz` with the same script. GitHub cant actually block a push on our plan, so a bad push still goes through but gets a red X and the person who pushed gets an email. Dont skip the hook with `--no-verify`.

Merges are fine (Pedro updates, merge conflict commits), except `Merge branch 'Biobuzz' of ...` / `Merge remote-tracking branch 'origin/Biobuzz'`, those come from a plain `git pull` and get refused.

### If your commit gets refused
Bad message, the commit wasnt made, just run it again with a fixed message:
```bash
git commit -m "D11 Regular Meet: what you changed"
```

Already committed with a bad message and havent pushed yet, fix the last commit's message:
```bash
git commit --amend -m "D11 Regular Meet: what you changed"
```

`git pull` got refused (`merges an old copy of Biobuzz back in`), undo the merge and pull with rebase instead:
```bash
git merge --abort
git pull --rebase
```

---
## Branches

`Biobuzz` is the default branch and the only branch on our repo. All current code lives there, so thats the one you clone, pull and push.

There is no `main` anymore. It used to be a template branch that mirrored Pedro's repo, but it was deprecated and deleted. It was the exact same commit as Pedro's `upstream/master`, so if we ever want it back:
```bash
git fetch upstream
git branch main upstream/master
```

Template/Pedro updates now get merged straight into `Biobuzz`, see "Pulling Updates From Pedro Repo" below.

---

## Branch Switching (checkout)

Switch to existing branch, this is when you already have the branch and just wanna hop onto it:
```bash
git checkout branch-name
```

Make new branch and switch to it, use this when you need a brand new branch that doesnt exist yet:
```bash
git checkout -b new-branch-name
```

See what branches exist, shows a list so you can double check the name before switching:
```bash
git branch
```

---

## Pulling Updates From Pedro Repo

Still relevant when we want fresh pedro code without losing our stuff.

1. Stash your work, this saves your uncommitted changes so they dont get in the way:
```bash
git stash
```

2. Grab the latest from Pedro (their branch is `master`, the old `pedro3` branch got merged into it and deleted):
```bash
git fetch upstream
```

3. Merge it straight into Biobuzz (there is no `main` in between anymore):
```bash
git checkout Biobuzz
git merge upstream/master --allow-unrelated-histories
```
`Biobuzz` and Pedro's `master` dont share any history, so git refuses the merge without `--allow-unrelated-histories`, and expect a LOT of conflicts the first time. Dont do this right before a comp, and check with whoever manages the repo first.

4. Bring your stashed work back:
```bash
git stash pop
```

### If conflicts happen
Android Studio's merge tool can get stuck (sometimes only leaves an Abort button). If that happens, resolve from terminal instead:

To take our own changes and ignore theirs:
```bash
git checkout --ours .
git add .
git commit -m "merge conflict resolved, kept ours"
```

To take theirs and overwrite ours (use this when pulling a Pedro update and we just want their version):
```bash
git checkout --theirs .
git add .
git commit -m "merge conflict resolved, kept theirs"
```

If you just want to bail out of the merge completely and go back to before you started:
```bash
git merge --abort
```

---

## Daily Essentials

Check whats changed / whats staged, run this before you do basically anything else so you know where you stand:
```bash
git status
```

Stage stuff to commit, this is everything in the current folder and below:
```bash
git add .
```
or a specific file, safer if you only want certain changes going in:
```bash
git add filename
```

Actually commit, the message goes in quotes after -m (see Commit Message Format above):
```bash
git commit -m "D11 Regular Meet: x was fixed"
```

Push to remote, sends your commits up to github:
```bash
git push
```

Pull latest from remote, grabs whatever teammates pushed and puts your new commits on top (rebase, see Repo Setup above). Uncommitted changes get stashed and brought back automatically:
```bash
git pull
```

Save uncommitted work without committing (temporary save, useful before switching branches):
```bash
git stash
```
bring it back later, applies the most recent stash and removes it from the stash list:
```bash
git stash pop
```

See commit history, --oneline keeps it to one line per commit so its easier to scan:
```bash
git log --oneline
```
 
---

## Less Common but Useful

See what actually changed before committing, shows line by line diffs:
```bash
git diff
```
just staged changes, same thing but only for whats already added:
```bash
git diff --staged
```

Undo changes to a file (careful, this deletes edits and cant be undone):
```bash
git checkout -- filename
```

Undo a commit safely (makes a new commit that reverses it, keeps history, good for stuff already pushed):
```bash
git revert commit-hash
```

Undo a commit but keep changes staged (use this if you commited too early or with wrong message but the actual code changes are still good, moves your branch pointer back one commit while everything you changed stays staged, ready to re-commit):
```bash
git reset --soft HEAD~1
```

Undo a commit AND unstage the changes (changes stay in your files, just not staged anymore, use this if you want to review/edit files more before staging again):
```bash
git reset HEAD~1
```

Undo a commit and throw away the changes completely (careful, this deletes your work, only use if you're sure you dont want those changes at all anymore):
```bash
git reset --hard HEAD~1
```

Force overwrite a local branch to exactly match a remote branch (careful, throws away any local commits that arent on the remote):
```bash
git reset --hard origin/Biobuzz
```

Merge one branch into the current one, brings in all the commits from other-branch:
```bash
git merge other-branch
```

Remove a tracked file from git (and optionally folder), this deletes it from disk too:
```bash
git rm filename
```
remove from git but keep the file locally, good if you accidentally committed something like a build folder:
```bash
git rm --cached filename
```