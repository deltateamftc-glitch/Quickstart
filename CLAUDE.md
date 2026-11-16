# deltateamftc — FTC robot code (notes for Claude Code)

Claude Code reads this file automatically. It holds what every session needs to know, on any computer.

## Who we are and how to work with us

- Rookie FIRST Tech Challenge team "deltateamftc", 2026–27 season. We are new to Android Studio, Java and the FTC SDK.
- Explain Android Studio and Driver Station (DS) steps **click by click**, and give the "why" in plain terms.
- Windows 11, PlayStation-style gamepad (`cross`/`circle`/`square`/`triangle`/`options`/`ps`), `gamepad1` only.
- **We edit files in Android Studio between chats.** Always re-read a file right before editing it. Treat changes you find on disk as intentional; if one looks like a bug, say so instead of reverting it.
- **Ask before pushing to GitHub.** Commit only when we ask (or after a change is verified on the robot).
- Verify SDK APIs from the real sources, not from memory or last season (see "Tooling").
- Competition OpMode: `TeamCode/.../Final/MainTeleOp.java`. **New ideas are tried in `experimental/TestTeleOp.java` first**, never straight in Final.
- Files in `Final/` have no `@Disabled`; files in `experimental/` get `@Disabled` (except the ones being tested right now).

## Documents to read at the start of a chat

- `FEATURE_LIST.md` (this folder): every feature with a one-line description. **Keep it updated** whenever a feature is added, changed, removed, or verified on the robot (then drop "(untested)").
- `FTC_HANDOFF.md` (this folder): the full history: design decisions, failed approaches (do not repeat them), current state. Add a short dated note to it when something important changes. It is a snapshot, so check it against the code and `git status`. Paths in it say `C:\Users\USER\...`, which is the team's first computer; on another computer use that computer's paths.

## Robot configuration (lives only in the DS config, not in code)

- Drive (goBILDA Yellow Jackets): `mec0` = Control Hub port 0 = front-left, `mec1` = Expansion Hub port 1 = front-right, `mec2` = Control Hub port 2 = rear-left, `mec3` = Expansion Hub port 3 = rear-right. Left side (mec0, mec2) is REVERSE in software. If the team rewires, re-run `experimental/MotorIdTest` before trusting this.
- IMU: config name `imu`, type "REV 9-Axis IMU" (external BNO055, I2C 0x29), mounted logo UP / I2C port FORWARD. May change later.
- Mechanisms (REV UltraPlanetary motors): `m0` = main intake, `m1` = second intake wheel (both REVERSE in Main), `m2` = the one shooter motor (one-shooter design since 2026-11-16; `m3` is gone), 1:1, holds 3000 rpm with its encoder. The user said it is on Control Hub port 2, which is also listed for `mec2` above: check the DS config.
- Servo: `s0` = Expansion Hub servo port 0, goBILDA Torque servo, DS config type "Servo". CROSS switches it between two positions (0 and 1 for now, to be tuned).
- Webcam: "Webcam 1", 320×240 MJPEG. This season's game pieces are lime-yellow balls.

## Versions (do not change by hand)

- FTC SDK 12.0.0, Pedro Pathing 3.0.1 (installed, not configured: no odometry yet), Android Gradle Plugin 8.13.2, Gradle 9.1.0.
- Upgrade only by merging Pedro's quickstart: `git fetch upstream` + `git merge upstream/master` (remote `upstream` = https://github.com/Pedro-Pathing/Quickstart.git). Never accept Android Studio's "AGP Upgrade Assistant".

## Tooling

- Repo: https://github.com/deltateamftc-glitch/Quickstart. `master` = robot-verified code; work happens on branches (see "Working on two computers"). Commit author for this repo (repo-local, keeps real emails private): `deltateamftc-glitch <deltateamftc-glitch@users.noreply.github.com>`.
- Compile check (Git Bash, about 1 minute, exit 0 = OK; ignore the "source value 8 is obsolete" and deprecated-API warnings):
  `JAVA_HOME="/c/Program Files/Android/Android Studio/jbr" ./gradlew --offline -q :TeamCode:compileDebugJavaWithJavac`
  (Leave out `--offline` the first time on a new computer.)
- SDK sources for checking APIs: `~/.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc/<Module>/12.0.0/*/<Module>-12.0.0-sources.jar`; read one class with `unzip -p <jar> path/To/Class.java`.
- Deploy: Android Studio run configuration **TeamCode**, device "REV Robotics Control Hub v1.0", then Run ▶.
- Over Wi-Fi: join the robot's Wi-Fi, then run `connect_robot.cmd` (in this folder; there is a shortcut on the desktop). It runs `adb connect 192.168.43.1:5555` so the Control Hub appears in Android Studio.
- Python may not be installed. Use PowerShell or Git Bash tools.

## Setting up a new computer

When the user asks you to set this project up on a new computer, do these steps in order. Tell the user what you are doing in simple words, and give click-by-click instructions for the steps only they can do.

1. **Git:** check `git --version`. Claude Code on Windows already needs Git for Windows, so it is usually there. If not: `winget install --id Git.Git -e`.
2. **Android Studio:** check whether `C:\Program Files\Android\Android Studio` exists. If not: `winget install --id Google.AndroidStudio -e` (ask the user to accept the Windows admin prompt).
3. **Android SDK (the user does this):** ask the user to open Android Studio once and finish the Setup Wizard: **Standard** install, accept all licenses (click each license on the left and choose **Accept**), **Finish**, and wait for the downloads. Continue when `%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe` exists.
4. **Get the code:** clone into `%USERPROFILE%\StudioProjects\Quickstart` (skip if it is already there):
   `git clone https://github.com/deltateamftc-glitch/Quickstart.git "$USERPROFILE/StudioProjects/Quickstart"`
   If GitHub asks to sign in, the user signs in themselves in the browser window that opens (never type passwords for them).
5. **Repo settings** (inside the Quickstart folder):
   - `git config user.name "deltateamftc-glitch"` and `git config user.email "deltateamftc-glitch@users.noreply.github.com"`
   - `git remote add upstream https://github.com/Pedro-Pathing/Quickstart.git` (skip if it exists)
6. **SDK location:** create `local.properties` in the Quickstart folder (it is git-ignored) with one line, using the real user folder, with `\:` and `\\` escapes, e.g.
   `sdk.dir=C\:\\Users\\<name>\\AppData\\Local\\Android\\Sdk`
7. **First build** (needs internet, can take 5–15 minutes; run it in the background):
   `JAVA_HOME="/c/Program Files/Android/Android Studio/jbr" ./gradlew :TeamCode:assembleDebug`
   If it fails because an SDK package or license is missing, read the error, and either ask the user to install it in Android Studio (**Tools → SDK Manager**) or install it with the SDK's `sdkmanager`. "BUILD SUCCESSFUL" = done.
8. **Desktop shortcut for the robot Wi-Fi script:** create a shortcut called "Connect robot" on the user's desktop that points to `connect_robot.cmd` in the Quickstart folder. Use `[Environment]::GetFolderPath('Desktop')` (the desktop may be inside OneDrive) and `WScript.Shell` `CreateShortcut`, with the Quickstart folder as the working directory.
9. **Open in Android Studio (the user does this):** **File → Open**, pick the `Quickstart` folder, click **Trust Project**, and wait for "Gradle sync" at the bottom to finish. If Android Studio offers to upgrade the Android Gradle Plugin, click **Don't ask again / Cancel** (see "Versions"). Check that **TeamCode** is selected in the run configuration dropdown at the top.
10. **Test the robot connection (the user does this):** join the robot's Wi-Fi, double-click "Connect robot" on the desktop; it should say `connected to 192.168.43.1:5555` and list the device. Then the Control Hub appears in Android Studio's device dropdown, and Run ▶ deploys the code.
11. Read `FEATURE_LIST.md` and `FTC_HANDOFF.md`, then tell the user the setup is finished and what they can do next.

## Working on two computers: branches (since 2026-11-16)

The team works in parallel on separate branches and merges finished work into `master`.

- **`master` = the good code**: what has worked on the robot and what we would take to a match. Nobody works directly on `master`.
- **Every new job gets its own branch**, started from an up-to-date `master`: `feature/<what-it-is>`, e.g. `feature/point-turning`, `feature/servo-arm`. Claude cloud sessions get a `claude/...` branch automatically; treat it like a feature branch.
- **Commit and push the branch often** (after asking), so the work is on GitHub and the other computer can open it.
- **Merge into `master`** only once it compiles and works on the robot: open a Pull Request on GitHub (branch → `master`), check the changes, **Merge**. Then everybody switches to `master` and pulls.
- **Parallel work:** two computers on two different branches never get in each other's way. Git only complains at merge time if both branches changed the same lines (a "merge conflict"); Claude can resolve those.
- **Keeping a long branch fresh:** if `master` got new merged work, merge `master` into the branch (Android Studio: on the branch, **Git → Merge… → `master`**).
- The robot runs whatever branch was deployed last. Telemetry or a quick look at **Git → Branches** shows which one is checked out.
- Before starting work: **Git → Pull** (or `git pull`) on the branch you are on.
- **Android Studio clicks** (branch menu = branch name in the top-left toolbar, or bottom-right in older versions):
  - New branch: switch to `master` and pull first, then branch menu → **New Branch…** → name → **Create**.
  - Open a branch made on another computer: **Git → Fetch**, then branch menu → **Remote Branches** → `origin/<name>` → **Checkout**.
  - Push: **Git → Push…** (Ctrl+Shift+K) → **Push**.
  - Commit any unsaved work (or ask Claude) before switching branches.
