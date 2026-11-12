# FTC Programming Handoff — deltateamftc (snapshot 2026-10-12)

> Rookie FIRST Tech Challenge team, 2026–27 season, new to Android Studio + Java + FTC SDK.
> Explain Android Studio / Driver Station (DS) steps click-by-click and give the "why" in plain terms.
> Windows 11, Android Studio (bundled JBR = JDK 25), PowerShell + Git Bash. PlayStation-style gamepad (`cross`/`circle`/`square`/`triangle`/`options`), `gamepad1` only so far.
> **The team edits files directly in Android Studio between chats. Always re-read a file before editing it and treat on-disk changes as intentional.**
> **Feature list:** `C:\Users\USER\StudioProjects\Quickstart\FEATURE_LIST.md` (repo root, created 2026-10-12). It lists every feature we've built, with a one-line description each. Read it at the start of a chat. **Update it whenever a feature is added, changed, removed, or verified on the robot** (drop the "(untested)" tag). Keep it to a list plus short descriptions.

---

## 1. Repository & File Context

- **GitHub:** https://github.com/deltateamftc-glitch/Quickstart (branch `master`)
  - Fork of https://github.com/Pedro-Pathing/Quickstart, which tracks https://github.com/FIRST-Tech-Challenge/FtcRobotController
- **Local clone:** `C:\Users\USER\StudioProjects\Quickstart`
- **Git state (2026-10-12):** HEAD `9865f7f` = `origin/master` (last push 2026-10-01). **The 2026-10-07…12 work is NOT committed yet:** modified `Final/MainTeleOp.java` and `experimental/BallDetectTeleOp.java`, new `WhiteBalanceProcessor.java` and `FEATURE_LIST.md`. It compiles but is mostly untested on the robot (see §5). Commit once the user confirms it works on the robot, and ask before pushing. Remote `upstream` = Pedro-Pathing/Quickstart.
  - Repo-local git author: `deltateamftc-glitch <deltateamftc-glitch@users.noreply.github.com>` (keeps the real email private).
  - Only the IDE-generated `gradle/gradle-daemon-jvm.properties` is intentionally left untracked (listed in `.git/info/exclude`; it was pushed by mistake in `023f598` and untracked again on 2026-11-12).
  - Old pre-upgrade backup: `C:\Users\USER\StudioProjects\Quickstart_backup_2026-09-30`.
  - Commit and push after each robot-verified change. Ask the user before pushing.
- **Code root:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` (below, `…/` = this root)

| Path | Package | Role | On DS? |
|---|---|---|---|
| `…/Final/MainTeleOp.java` | `teamcode.Final` | **Competition TeleOp, rewritten 2026-11-11 (untested):** field-centric sin/cos mecanum + heading hold, HOME (PS) ball grab (search / approach / push / lost-ball chase, YELLOW only since 2026-11-11), DPAD DOWN intake, DPAD UP shooting sequence, shooters m2/m3 at a constant 1000 rpm. Details in §2 Controls | **Yes** — "Main TeleOp", group "Final" |
| `…/experimental/TestTeleOp.java` | `teamcode.experimental` | **Test bench (created 2026-11-11):** copy of the old MainTeleOp (drive, ✕ auto-turn, SHARE, m0/m1 intake toggles, Y unjam, m0 amps). New ideas are tried here before going into Main | **Yes** (not `@Disabled`) — "Test TeleOp", group "Test" |
| `…/PIDController.java` | `teamcode` | Shared PID class, used by MainTeleOp and experimental SinCos | n/a |
| `…/WhiteBalanceProcessor.java` | `teamcode` | Shared `VisionProcessor`: software auto white balance, used by BallDetectTeleOp (new 2026-10-07, uncommitted) | n/a |
| `Quickstart/FEATURE_LIST.md` (repo root) | — | List of all features + one-line descriptions. Keep it updated | n/a |
| `…/experimental/MecanumTeleOpSinCos.java` | `teamcode.experimental` | Original of MainTeleOp. Identical except package, class name, and `@TeleOp` name | No (`@Disabled`) — "Mecanum Field-Centric (Sin/Cos)" |
| `…/experimental/MecanumTeleOp.java` | experimental | Robot-centric mecanum using the linear formula (`denominator`) | No — "Mecanum TeleOp" |
| `…/experimental/MotorIdTest.java` | experimental | Port ID test: R-bumper = mec0 25% / mec2 50% / mec1 75% / mec3 100%. Single motor at 50%: cross = mec0, square = mec2, triangle = mec1, circle = mec3 | No — "Motor ID Test" |
| `…/experimental/ImuAccelerationTest.java` | experimental | BNO055 linear + overall acceleration via raw I2C registers | No — "IMU Acceleration Test" |
| `…/experimental/CircleButtonTest.java` | experimental | First hello-world (hold circle → telemetry line) | No — "Circle Button Test" |
| `…/experimental/SquareButtonTest.java` | experimental | Hold square → "Square is pressed" in telemetry (new 2026-10-12, uncommitted, untested) | **Yes** (not `@Disabled` yet, so the user can try it) — "Square Button Test", group "Test" |
| `…/experimental/BallDetectTeleOp.java` | experimental | Copy of MainTeleOp + webcam ("Webcam 1", 320×240 **MJPEG**; this webcam has no 320×240 in the default YUY2) purple + green ball detection. Two `ColorBlobLocatorProcessor`s (`ARTIFACT_PURPLE` with yellow circle overlay, `ARTIFACT_GREEN` with red overlay) on one portal, filtered by area 50–10000 px (team-tuned) and circularity ≥ 0.6. Results are merged into a `Ball(color, blob)` list sorted by contour area, largest = closest first. Telemetry reports each ball's color and 3×3 grid cell (e.g. "bottom left"). **Hold CIRCLE = auto-aim** at the closest ball: `AIM_KP 0.001` (team-tuned, power/px) + `AIM_MIN_POWER 0.08`, stops within 10 px, max 0.4, right stick overrides, cancels ✕ auto-turn. Purple detection + aiming verified on robot 2026-09-30; green + closest-ball verified on robot 2026-10-01. **HOME/PS (`psWasPressed`) cycles modes NORMAL → FOLLOW → HIT**, with rumble feedback (1/2/3 blips). Follow mode: GREEN ONLY (team has a purple sofa; normal mode still does both colors), max area 76800. It turns with the aim PID and drives robot-centric forward/back to keep `FOLLOW_DISTANCE_CM 10` (from the lens). `FOLLOW_KP 0.01`/cm, min 0.08, max 0.3, ±2 cm tolerance; forward power is scaled down when the ball is off-center. Distance = `CALIBRATION_DISTANCE_CM 30 × CALIBRATION_RADIUS_PX 48 / radius`; the 48 is an uncalibrated guess. A ball touching the top or bottom of the picture → no driving, because its circle fit is truncated. Any stick > 0.1 pauses following. CROSS is ignored in follow mode. Follow mode verified on robot 2026-10-01 (team set `FOLLOW_TOLERANCE_CM 5`). **HIT mode** (verified on robot 2026-10-01; push then raised to 0.6 power / 1.0 s, untested): GREEN ONLY; `HitState` READY → APPROACH (aim PID + `HIT_APPROACH_POWER 0.35`) → PUSH (when closer than `HIT_PUSH_START_CM 20`, touching the top/bottom edge, or lost within `HIT_LOST_NEAR_CM 40` after a 0.3 s grace: drive straight at `HIT_PUSH_POWER 0.6` for `HIT_PUSH_SECONDS 1.0`, holding the IMU heading via headingPid) → DONE. TRIANGLE (`triangleWasPressed`) → SEARCH: sweeps ±`SEARCH_SWEEP_DEG 90` at 0.25 turn power around the starting heading and goes to APPROACH when a ball is seen; TRIANGLE again stops. Sticks cancel → DONE. `aimTurn()`/`followDrive()` are instance methods that auto-reset their PID if it was unused the previous loop (`loopCount`). | **Yes** (intentionally NOT `@Disabled` while testing) — "Ball Detect TeleOp", group "Experimental" |
| `…/pedro/Constants.java` | `teamcode.pedro` | Pedro 3.0 follower factory. **Unconfigured: `create()` returns `null`** | — |
| `…/pedro/Tuning.java` | pedro | Empty placeholder | — |
| `…/pedro/procedures/*.java` | `pedro.procedures` | Pedro 3.0 AutoTune `Procedure`s (Mecanum, Pinpoint, OTOS, OctoQuad, ThreeWheel, ThreeWheelIMU, TwoWheel, Foresight, Tests). Run from Pedro's web AutoTune page, not as DS OpModes | — |

**Build config.** Matches the Pedro quickstart; do not edit.
- `build.dependencies.gradle`: FTC SDK **12.0.0** (Inspection, Blocks, RobotCore, RobotServer, OnBotJava, Hardware, FtcCommon, Vision), `com.pedropathing:revhub:3.0.1`, `com.pedropathing:tuning:1.0.1`, maven `https://repo.dairy.foundation/releases/`.
- `build.gradle`: AGP **8.13.2** via the `plugins {}` block. Gradle wrapper **9.1.0**. compileSdk 34, Java 8 source/target.
- **Module roles flipped this season:** `TeamCode/build.gradle` = `com.android.application` (the runnable app, depends on `:FtcRobotController`). `FtcRobotController/build.gradle` = `com.android.library`.

---

## 2. Hardware & Architecture Constraints

### Robot configuration (active config on the DS)
| Config name | Hub / motor port | Wheel | Software direction |
|---|---|---|---|
| `mec0` | Control Hub 0 | front-left | `REVERSE` |
| `mec1` | Expansion Hub 1 | front-right | FORWARD (default) |
| `mec2` | Control Hub 2 | rear-left | `REVERSE` |
| `mec3` | Expansion Hub 3 | rear-right | FORWARD (default) |

- The port→wheel map was found empirically with MotorIdTest because the wiring is messy. It exists **only** in the DS config, not in code. Re-run the test after any rewiring.
- All drive motors use `ZeroPowerBehavior.BRAKE`. Strafing works, so the wheel X-pattern is correct.
- **IMU:** config name `imu`, type **"REV 9-Axis IMU"**. This is an external BNO055 board → SDK class `com.qualcomm.hardware.rev.Rev9AxisImu` (extends `BNO055IMUNew`, I2C 0x29).
  - Orientation in code: `new Rev9AxisImuOrientationOnRobot(LogoFacingDirection.UP, I2cPortFacingDirection.FORWARD)`. Verified: yaw is CCW-positive and field-centric drive is correct.
  - The user expects the IMU config or orientation may need to change later. Only the two constants would change.
- **No odometry hardware yet** (no Pinpoint, OTOS, or dead wheels), so there is no position tracking.

### Controls (MainTeleOp, gamepad1) — rewritten 2026-11-11, compiles, UNTESTED on the robot
The user said they had been testing in the Final OpMode by mistake, so on 2026-11-11 the old MainTeleOp was copied to `experimental/TestTeleOp` ("Test TeleOp", group "Test") and `Final/MainTeleOp` was rewritten to contain only what the robot needs in a match. New or untested ideas now go to TestTeleOp first.
- Left stick = drive / strafe (`-left_stick_y` forward), right stick X = turn (+ = clockwise). Always field-centric (driver's perspective) with heading hold. There is NO stabilization switch and NO ✕ auto-turn in Main anymore (the user said "this should be the only options").
- **OPTIONS** (held) → `imu.resetYaw()`. Point the robot away from the driver first. Yaw also resets at INIT.
- **HOME (PS)** → ball grab (`psWasPressed`; the user chose HOME on 2026-11-11, same button Ball uses to switch modes). Starts SEARCH (turns ±`SEARCH_SWEEP_DEG` 90° at 0.25 until a yellow ball is seen; if one is already in view it goes straight to APPROACH). HOME again or any stick (|x|>0.1) cancels. States `HitState`: IDLE → SEARCH → APPROACH (aim PID + `HIT_APPROACH_POWER 0.35`) → PUSH (`HIT_PUSH_POWER 0.6` for `HIT_PUSH_SECONDS 1.0`, holding the heading with its own `pushPid`, constants `PUSH_HEADING_*` = the old Ball values 0.02/0.02/0.002) → IDLE. **LOST** (new in this flow; this is the user's "overshoot searching", ported from FOLLOW's lost-ball chase): ball gone after the 0.3 s grace and last seen farther than `HIT_LOST_NEAR_CM 40` → turns toward the side it left (`LOST_TURN_POWER 0.15`, waits without turning if it vanished within `LOST_EDGE_PX 40` of center) for up to `LOST_SECONDS 1.5` / `LOST_MAX_TURN_DEG 120`, then IDLE ("lost the ball"); sees it again → APPROACH. Unlike Ball's HIT there is no READY state: the robot never drives by itself until TRIANGLE is pressed. Yellow only (`BALL_COLOR = ColorRange.YELLOW`, new season, color range untested), area 200–76800, circularity ≥ 0.6, biggest blob = closest. No color fix / camera white balance / purple / CIRCLE-aim / FOLLOW in Main.
- **DPAD DOWN** → intake toggle: OFF ↔ COLLECTING. `m0` = `INTAKE_POWER 0.7` forward (+), `m1` stays still; `M0_DIRECTION`/`M1_DIRECTION` are REVERSE since 2026-11-11 (user said both wheels spun the wrong way). Ignored while SHOOTING.
- **DPAD UP** → shooting sequence (m0 first pushes inward `HICCUP_START_FORWARD_SECONDS 0.25` so the first ball does not slip out, then the hiccups; added 2026-11-11): OFF/COLLECTING → SHOOTING, SHOOTING → OFF. In SHOOTING `m1` = `M1_POWER 0.7` constantly (the user did not say which direction is right, so the sign is a guess; make it negative to flip), and `m0` does hiccups of −`HICCUP_BACK_POWER 0.7` for `HICCUP_BACK_SECONDS 0.1` (team-tuned) then +`HICCUP_FORWARD_POWER 0.3` for `HICCUP_FORWARD_SECONDS 0.5` (user lowered it from 0.7 on 2026-11-11); the first inward push uses `HICCUP_START_POWER 0.7`. M0_DIRECTION and M1_DIRECTION are REVERSE, so "forward/inward" is the physical inward direction, repeated for the whole sequence (user correction 2026-11-11: first version did only 5 and stopped), until DPAD UP is pressed again, which stops m0 and m1. Nothing starts the intake automatically during a ball grab.
- **Shooter wheels `m2`, `m3`** (REV UltraPlanetary HD Hex motors with encoders; m0/m1 are the same type, the four drive motors are goBILDA Yellow Jackets): hub velocity control, `setVelocity(SHOOTER_RPM 1000 / 60 × 28 × SHOOTER_GEAR_RATIO)` ticks/s, once after START (never in INIT, FTC rules). `setVelocityPIDFCoefficients(P 30, I 5, D 0, F = 32767 / (6000/60×28) ≈ 11.7)`; P/I are untuned starting guesses for the "aggressive" PID the user wants (balls slow the wheels). **`SHOOTER_GEAR_RATIO` = 1.0 is REAL: the user confirmed a 1:1 gear ratio on 2026-11-11.** `M3_DIRECTION = REVERSE` is a guess for opposing wheels. If a shooter shows < 10 % of target speed `SHOOTER_ENCODER_CHECK_SECONDS 1.0` s after START (encoder not connected yet), it switches to `RUN_WITHOUT_ENCODER` with plain power = target/max and telemetry says "NO ENCODER". Telemetry shows both rpm and "AT SPEED" (within 3 %).
- Hub bulk caching is AUTO in Main (`LynxModule.BulkCachingMode.AUTO`) for faster loops. Telemetry is short on purpose: intake, shooter, ball, heading.
- DS config needs motors `m0`, `m1`, `m2`, `m3` (plain names; don't confuse them with the drive motors `mec0…mec3`) or INIT fails.
- **TestTeleOp controls** (= the old Main): same drive + OPTIONS + CROSS auto-turn + SHARE stabilization; DPAD DOWN/UP = m0 toggle (tap = keeps spinning, again = stop, other = reverse); DPAD RIGHT/LEFT = same for m1; TRIANGLE (Y) = m0 unjam (+ for `UNJAM_FIRST_SECONDS 0.08` (team-tuned), then − for 0.5 s, then back to the toggle state); m0 current (A) + peak telemetry. Not `@Disabled`.
- **SHARE** (Ball and Test only, added 2026-10-12, untested) → stabilization on/off. "Stabilization" = field-centric (headless) drive + heading hold. Off = robot-centric and no hold. Rumble: 300 ms = on, 1000 ms = off. `STABILIZATION_ON_AT_START true`. ✕ auto-turn and the ball modes still work when it's off.

### Extra controls (BallDetectTeleOp only, gamepad1)
- Same drive, OPTIONS, and CROSS as the old MainTeleOp / TestTeleOp (CROSS only in NORMAL mode).
- **Hold CIRCLE** (NORMAL mode) → turn toward the closest ball (purple or green).
- **HOME/PS** → next mode: NORMAL → FOLLOW → HIT (1/2/3 rumble blips).
- **TRIANGLE** (HIT mode) → start searching / stop searching.
- Any stick movement overrides the automation (FOLLOW: pauses; HIT: cancels → DONE).

### Vision setup
- Webcam config name `"Webcam 1"` (default), on the Control Hub's USB 3.0 port. A standard USB webcam.
- **Must use `.setStreamFormat(VisionPortal.StreamFormat.MJPEG)`.** This webcam offers 320×240 only in MJPEG; YUY2 throws `OpenCvCameraException: Camera does not support requested resolution`.
- SDK 12.0 built-ins used: `ColorBlobLocatorProcessor`, `ColorRange.ARTIFACT_PURPLE` / `ARTIFACT_GREEN` (DECODE 2025–26 balls, used for testing until this season's game pieces arrive), and `Blob.getCircle()` / `getCircularity()` / `getContourArea()` / `getContourPoints()`.
- Camera Stream for debugging: DS ⋮ → Camera Stream, only during INIT.
- The team has a **purple sofa**, so FOLLOW/HIT are green-only.
- A neural-net detector was discussed but not pursued. Options: Limelight 3A (SDK sample `SensorLimelight3A`, detector results), HuskyLens (`SensorHuskyLens`), or self-bundled TFLite (TFOD was removed from the SDK; slow on the Control Hub). Recommendation: stay with color detection.

### Core math / architecture decisions
- `LinearOpMode` style. Tunables are `static final` constants at the top of each class.
- **Sin/cos (polar) mecanum:**
  `theta = atan2(y,x) - heading; power = hypot(x,y); s = sin(theta-π/4); c = cos(theta-π/4); max = max(|s|,|c|)`
  `fl = power*c/max + turn; fr = power*s/max - turn; rl = power*s/max + turn; rr = power*c/max - turn`
  If `power + |turn| > 1`, divide all four by it. **Field-centric = the single `- heading` subtraction.**
- **Heading PID works in degrees:**
  - `errorDeg = AngleUnit.normalizeDegrees(0 - toDegrees(heading))`
  - `turn = Range.clip(-pid.update(errorDeg), -MAX_AUTO_TURN_POWER, MAX_AUTO_TURN_POWER)`
  - The sign is flipped because +turn = CW but +yaw = CCW.
- **Auto-turn ends only when settled:** `|errorDeg| < HEADING_TOLERANCE_DEG` (1° in MainTeleOp since 2026-10-01; still 2° in BallDetectTeleOp) **AND** `|imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate| < 10`.
- **`PIDController(kP, kI, kD, integralZone)`:**
  - `update(error)` gets dt from `ElapsedTime`.
  - The first call after `reset()` returns P only, which avoids a stale-dt / derivative spike.
  - The integral accumulates only while `|error| < integralZone` and is zeroed otherwise (anti-windup).
  - D is the finite difference of the error.
- **Code organization:**
  - `Final/` = competition OpModes (visible on the DS).
  - `experimental/` = everything else, all `@Disabled`.
  - The folder is `Final` with a capital F because `final` is a Java keyword.
  - `PIDController` stays in the root because it is shared.
  - DS display names must be unique.
- **IMU acceleration:**
  - The SDK `IMU` interface has no acceleration methods. But the SDK inits the BNO055 in `SensorMode.IMU` (fusion) with m/s² units, so the registers are live.
  - Read them directly: `rev9AxisImu.getDeviceClient().read(reg, 6)`, then little-endian int16 / 100.0 = m/s².
  - `BNO055IMU.Register.LIA_DATA_X_LSB` (0x28) = acceleration with gravity removed. `ACC_DATA_X_LSB` (0x08) = acceleration including gravity.
  - The axes are the chip's own axes (printed on the board), not the robot frame. No config change was needed.
  - **Acceleration is not usable for position** (double-integration drift). Pedro needs odometry.

### Tooling notes (for Claude)
- **Deploy:** Android Studio run configuration **`TeamCode`** (Android App, module `Quickstart.TeamCode`), device "REV Robotics Control Hub v1.0", then Run ▶.
- **CLI compile check** (Git Bash, ~1 min, exit 0 = OK; ignore the Java-8-obsolete and deprecated-API warnings):
  `cd C:/Users/USER/StudioProjects/Quickstart && JAVA_HOME="/c/Program Files/Android/Android Studio/jbr" ./gradlew --offline -q :TeamCode:compileDebugJavaWithJavac`
- **Verify SDK APIs from source, not memory:** `~/.gradle/caches/modules-2/files-2.1/org.firstinspires.ftc/<Module>/12.0.0/*/<Module>-12.0.0-sources.jar`. Read files with `unzip -p <jar> path/To/Class.java`. A bulk `unzip` into scratchpad once came out incomplete.
- The `gh` CLI is not installed. Use `curl https://api.github.com/...` for GitHub info.

---

## 3. Failed Approaches & Non-Solutions (do not repeat)
1. **Creating an "FtcRobotController" run configuration.** Impossible this season, because it's a library module and only `Quickstart.TeamCode` is selectable. The original "TeamCode" config was correct all along; the earlier advice assumed last season's layout. The original grey Run button was never explained and fixed itself, most likely because Gradle sync was still running.
2. **Reversing the right-side motors** (the usual assumption). Wrong for this robot: the left side (`mec0`, `mec2`) must be `REVERSE`.
3. **Forward/back on `right_stick_y`** (a team edit). Reverted: forward/back is `-left_stick_y`.
4. **Auto-turn v1 stopped the PID as soon as `|error| < 2°`.** Momentum carried the robot to ~7° past zero. Fixed by also requiring turn rate < 10°/s, plus raising `HEADING_KD` from 0.001 to 0.002.
5. **Expecting folders to hide OpModes on the DS.** The DS ignores folders. Use `@Disabled`.
6. **A package named `final`.** Illegal (Java keyword), so it's `Final`.
7. **Acceleration via `IMU` / `Rev9AxisImu` methods.** They don't exist: `BNO055IMUNew` implements only `IMU`, not `BNO055IMU`.
   - Alternative *not* used: reconfigure as "Adafruit BNO055 IMU" (`AdafruitBNO055IMU`, legacy `BNO055IMU` API, needs `parameters.i2cAddr = BNO055IMU.I2CADDR_ALTERNATE`). That would break all the `IMU`-interface code.
8. **Android Studio Upgrade Assistant (AGP 9.3.2) and earlier IDE auto-bumps (AGP 9.3.2 / Gradle 9.5.0).** Don't apply them. Stay on the Pedro quickstart's AGP 8.13.2 / Gradle 9.1.0.
9. ~~Waiting on SDK v12.0~~ Resolved 2026-09-30: upgraded by `git merge upstream/master` (Pedro quickstart added 12.0.0 + revhub 3.0.1 + tuning 1.0.1 on 2026-09-18). Future SDK/Pedro upgrades: `git fetch upstream && git merge upstream/master`; do not hand-edit version numbers. SDK 12.0 only breaks AprilTag code (singleton vs cluster detections).
10. **Webcam at 320×240 in the default YUY2 format.** It crashes at INIT. Use MJPEG (see Vision setup).

---

## 4. Current Implementation State
- ✅ **Verified by the user on the robot:**
  - field-centric sin/cos drive
  - OPTIONS heading reset
  - ✕ auto-turn to 0° with the settled check ("now it's good")
  - robot-centric experimental drive
  - MotorIdTest
  - ImuAccelerationTest (user reported working)
  - SDK 12.0 / Pedro 3.0.1 build on the robot ("it works on the robot, all good", 2026-09-30)
  - BallDetectTeleOp: detection, segments, CIRCLE aim, green, closest ball, FOLLOW, and HIT (before the push was strengthened)
- ✅ **Compiles** (CLI build verified after the last change).
- **(Old MainTeleOp, now `TestTeleOp`) constants:** `HEADING_KP 0.005` (team retune 2026-10-01, was 0.02), `HEADING_KI 0.02`, `HEADING_KD 0.002`, `HEADING_I_ZONE_DEG 5`, `HEADING_TOLERANCE_DEG 1` (was 2), `HEADING_SETTLED_DEG_PER_SEC 10`, `MAX_AUTO_TURN_POWER 0.6`.
- **(Old MainTeleOp, now `TestTeleOp`) telemetry:** auto-turn state, heading, heading error, turn rate, turn power, stick angle and magnitude, the four wheel powers.
- **Open issues / edge cases:**
  - ⚠️ Uncommitted work since 2026-10-07 (see Git state in §1 and the list in §5).
  - ⚠️ BallDetectTeleOp is NOT `@Disabled`, so it appears on the DS next to Main TeleOp. Add `@Disabled` before competitions (offered; the user hasn't answered).
  - ⚠️ `CALIBRATION_RADIUS_PX 48` was never measured. FOLLOW works acceptably anyway; calibrate if distances feel off.
  - HIT push strengthened to 0.6 power / 1.0 s on 2026-10-01; the user hasn't reported back yet.
  - BallDetectTeleOp is a third independent copy of the drive code (also still has `HEADING_KP 0.02` / tolerance 2), so MainTeleOp tuning doesn't carry over. Heading hold and the SHARE switch were added to both files separately; keep them in sync by hand.
  - MainTeleOp, TestTeleOp, BallDetectTeleOp and `experimental/MecanumTeleOpSinCos` are independent copies of the drive code, so changes don't carry over. The new MainTeleOp has no auto-turn constants (`HEADING_KP` etc. are gone; the push uses its own `PUSH_HEADING_*`).
  - OPTIONS reset triggers every loop while the button is held. Harmless.
  - A heading error crossing ±180° causes a one-loop derivative spike, which is clipped by `MAX_AUTO_TURN_POWER`. Harmless.
  - There is no strafe-correction multiplier (`x * 1.1`). Optional, if strafing feels weak.
  - Pedro Pathing is installed but unconfigured. `Constants.create()` returns `null`, so any Follower use will NPE. There is no odometry hardware yet.

---

## 5. Active Waypoint
- **Last committed work (2026-10-01, pushed):** BallDetectTeleOp with NORMAL / FOLLOW / HIT modes and TRIANGLE search, green-only FOLLOW/HIT, stronger HIT push (`e775de0`), and the team's MainTeleOp retune (`9865f7f`).
- **Everything below is UNCOMMITTED** (compiles; robot testing pending):
- **2026-10-07 (uncommitted, untested on robot):** FOLLOW lost-ball chase. When the green ball vanishes, the robot keeps turning toward the side it was last seen on (`FOLLOW_LOST_TURN_POWER 0.35`), for up to `FOLLOW_LOST_SECONDS 1.5` or `FOLLOW_LOST_MAX_TURN_DEG 120`. If it vanished within `FOLLOW_LOST_EDGE_PX 40` of the center, the robot waits without turning. Touching the sticks or switching modes cancels the chase. Problem it fixes: fast-rolling balls left the picture before the aim PID could turn (aim turn is slow: `AIM_KP 0.001`, max 0.4).
- **2026-10-07 (uncommitted, untested on robot):** white balance for yellow lighting in BallDetectTeleOp.
  - New shared helper `…/WhiteBalanceProcessor.java` (`teamcode` package, a `VisionProcessor`): software "gray world" auto white balance with per-channel gains clamped to 0.5–3 and smoothed 0.2 per frame. It is added FIRST in `addProcessors(colorFix, purple, green)`. VisionPortalImpl runs the processors in order on the same `Mat`, and the locator's cached `submat` needs the frame changed in place (`Core.multiply`), so the order matters. Frames are RGB(A). SQUARE toggles it (`COLOR_FIX_ON_AT_START true`).
  - Webcam's own white balance via SDK `WhiteBalanceControl` (`visionPortal.getCameraControl`, which only works while the camera is STREAMING): DPAD UP/DOWN = manual ±200 K, DPAD LEFT = auto. `CAMERA_WHITE_BALANCE_K 0` = auto at start (a positive value is applied at start). The mode and kelvin are cached in fields so the camera isn't polled every loop. If the webcam doesn't support it, telemetry says so.
  - `waitForStart()` was replaced with an `opModeInInit()` loop, so everything above can be adjusted during INIT with the Camera Stream open.
- **2026-10-08 (uncommitted):** BallDetectTeleOp's first telemetry line now shows the loop speed (loops/sec, averaged over 0.5 s, plus ms per loop) and the camera rate (`visionPortal.getFps()`).
- **2026-10-09 (uncommitted, untested on robot):** heading hold in MainTeleOp (copied to BallDetectTeleOp on 2026-10-12; not in SinCos).
  - Fixes the robot slowly rotating from uneven wheel drag when no turn is commanded.
  - When the right stick is released (and ✕ auto-turn is off), it waits until `|turnRate| < HOLD_CAPTURE_DEG_PER_SEC 20`, then saves the current heading.
  - Correction: `turn = -sign(err)·HOLD_KP 0.02·max(0, |err| - HOLD_DEADBAND_DEG 1) + HOLD_KD 0.001·turnRateDeg`, clipped to ±`MAX_HOLD_TURN_POWER 0.3`. It grows from 0, so nothing jumps at 1°. The rate term damps drift even inside the deadband.
  - OPTIONS, the right stick, or ✕ cancels it. `HEADING_HOLD_ENABLED` switches the whole feature off. The D term assumes `zRotationRate` is CCW-positive like yaw (should be true per the SDK, not yet confirmed on the robot).
- **2026-10-12 (uncommitted, untested):** the same heading hold copied into BallDetectTeleOp (same constants).
  - An `autoTurning` flag marks the loops where automation sets the turn power: CIRCLE aim, ✕ auto-turn, FOLLOW following/chasing, and HIT APPROACH/PUSH/SEARCH. Hold is only active when neither the driver nor automation is turning, so it does work in HIT READY/DONE and in FOLLOW while waiting.
  - Team edit seen: `FOLLOW_LOST_TURN_POWER` lowered 0.35 → 0.15.
- **2026-10-12 (uncommitted, untested):** SHARE stabilization switch in both TeleOps (see Controls). Created `Quickstart/FEATURE_LIST.md` (repo root, so it's visible in Android Studio's Project view and goes to GitHub with the code).
- **2026-11-11 (uncommitted, untested):** the intake experiments (m0 / m1 toggles, Y unjam, amps) were built in MainTeleOp by mistake, then moved to the new `experimental/TestTeleOp`. `Final/MainTeleOp` was rewritten from scratch as the match OpMode (see Controls): drive + heading hold, TRIANGLE ball grab, DPAD DOWN intake, DPAD UP shooting sequence (repeating hiccups), shooters m2/m3 at 1000 rpm. Compiles, nothing tried on the robot.
  - **Open questions for the user:** (1) [answered: gear ratio is 1:1], (2) which way m1 and the shooter wheels must spin (`M1_POWER` sign, `M3_DIRECTION`), (3) whether DPAD DOWN spinning m0 forward really is the "pick up" direction, (4) PID tuning of the shooter once the encoders are connected, (5) the old ✕ auto-turn / SHARE switch / color fix were left out of Main on purpose; they can be added back if wanted.
- **Free gamepad1 buttons now:** MainTeleOp: circle, cross, square, triangle, DPAD LEFT/RIGHT, bumpers, triggers, touchpad, SHARE. BallDetectTeleOp: DPAD RIGHT, bumpers, triggers, touchpad.
- **Waiting on the user:**
  - Robot test results for all the uncommitted features. Especially check the heading-hold "twist by hand" test: the robot must push back. If it helps the twist instead, the sign is wrong.
  - Then commit, and ask before pushing.
  - Feedback on the stronger HIT push.
  - Whether to `@Disabled` BallDetectTeleOp.
- **Likely next:**
  - This season's game pieces → new color ranges / detection tuning (the user is still waiting for the new balls).
  - Possibly move vision features into a competition OpMode in `Final/`.
  - Alliance-specific TeleOps (the user mentioned maybe "two teleops for two sides").
  - Autonomous OpModes (`@Autonomous`, in `Final/`, shown in the DS's left dropdown).
  - Pedro Pathing setup: needs odometry hardware first, then `pedro/Constants.java` and the AutoTune procedures.
- **Rule for new OpModes:** files in `Final/` have no `@Disabled`; files in `experimental/` get `@Disabled`. To run an experimental one, delete its `@Disabled` line and redeploy. Exception: BallDetectTeleOp is left enabled while it is being tested.

---

## 6. Kickoff Prompt (paste as the first message of the new chat)

We're a beginner FTC team (2026–27 season) coding Java in Android Studio on Windows. Before doing anything, please read our handoff document `FTC_HANDOFF.md` in the Quickstart repo root (`C:\Users\USER\StudioProjects\Quickstart\FTC_HANDOFF.md` on the first computer) completely. It has our full project context: repo, versions, robot config, controls, code design, what we already tried that failed, and the current state.

Short summary: the repo is https://github.com/deltateamftc-glitch/Quickstart (a fork of Pedro-Pathing/Quickstart, on FTC SDK 12.0.0 and Pedro Pathing 3.0.1), cloned locally at `C:\Users\USER\StudioProjects\Quickstart`. The Android Studio run configuration is "TeamCode".
- **Competition TeleOp:** `Final/MainTeleOp.java` (rewritten 2026-11-11, see §2 Controls). Field-centric mecanum with heading hold and an external REV 9-Axis IMU "imu". OPTIONS resets the heading; HOME = ball grab on/off; DPAD DOWN = intake; DPAD UP = shooting sequence (m0 hiccups repeatedly); shooters m2/m3 hold 1000 rpm. Test ideas go in `experimental/TestTeleOp.java` first.
- **Webcam OpMode:** `experimental/BallDetectTeleOp.java`. Uses "Webcam 1" (320×240 MJPEG) to detect balls. HOME switches between NORMAL, FOLLOW and HIT modes; FOLLOW and HIT only chase green balls.
- **Git:** everything up to 2026-10-01 is pushed. Newer work (heading hold, the SHARE stabilization switch, white balance, lost-ball chase, loop speed, the feature list) is NOT committed yet and mostly untested on the robot.

Our feature list is at `C:\Users\USER\StudioProjects\Quickstart\FEATURE_LIST.md`. Please read it too and keep it updated.

Important rules:
- Re-read files before editing them, because we also edit them in Android Studio.
- Check SDK APIs against the SDK 12.0 sources, not memory.
- Ask before pushing to GitHub.
- Explain Android Studio and Driver Station steps click by click.

Next I want to: ___

- **2026-11-11 (later, uncommitted, untested on robot):** MainTeleOp changes after the first robot tests: m0/m1 `DcMotorSimple.Direction` constants `M0_DIRECTION`/`M1_DIRECTION` = REVERSE (both intake wheels spun the wrong way); shooting sequence now starts with a 0.25 s inward push (`HICCUP_START_FORWARD_SECONDS`, first ball slipped out otherwise); the long inward part of each hiccup runs at 0.3 power (`HICCUP_FORWARD_POWER`); shooter gear ratio confirmed 1:1; **new season = yellow game pieces, so Main's `BALL_COLOR` is now `ColorRange.YELLOW`** (the SDK's built-in range, not tuned on the real piece; check the overlay on the DS and retune if it misses). Ball Detect TeleOp still uses green/purple.

- **2026-11-11 (ball grab rework, uncommitted, untested on robot):** user report: with `ColorRange.YELLOW`, HOME made the robot drive straight forward without searching. Likely cause: a large yellow-ish blob (wall/floor/light) passed the filters right away, and its size or touching the picture edge sent it straight to PUSH. Not confirmed. Changes in MainTeleOp: (1) `WhiteBalanceProcessor` color fix added first in `addProcessors(colorFix, ballLocator)`, always on (user: no button, SQUARE toggle removed the same day); (2) `BALL_MAX_AREA` 76800 → 30000; (3) SEARCH needs the target in view without a gap for `LOCK_CONFIRM_SECONDS 0.2` (robot stands still while confirming) before APPROACH; (4) `SEARCH_AGAIN_AFTER_HIT true`: after PUSH and after a failed LOST chase it goes back to SEARCH (new sweep center) until HOME or a stick; `hitsDone` counter; (5) `findTarget()` replaces `findClosestBall()`: per-ball neighbourhood score = total contour area of balls within `CLUSTER_DISTANCE_PX 100`; the best group is aimed at its area-weighted x, and distance/push use the group's largest blob (`Target{closest, aimX, count}`); (6) telemetry "camera sees" = raw blob count, biggest raw area + circularity, balls in group. Known limit: touching balls may merge into one non-round blob and get rejected by `BALL_MIN_CIRCULARITY 0.6`. Follow-up the same day: `AUTO_INTAKE_DURING_GRAB true` = m0 runs at `INTAKE_POWER` whenever `hitState != IDLE` and the intake state is OFF (SHOOTING still wins; the DPAD DOWN state is untouched, so it returns to that when the grab ends; uses the previous loop's hitState, one loop of lag).
  - **Later the same day:** robot test showed the robot never locked on (kept searching with a ball in front), and the DS camera stream looked very blue: the gray-world white balance overcorrected in the yellow-lit room (yellow balls / walls pull the average). `WhiteBalanceProcessor` got options, with defaults unchanged for Ball: `setIgnoreColorfulPixels(true)` measures the mean only on grayish pixels ((max−min) < 0.5·max, max > 30, needs ≥ 5 % of the picture or the old gains are kept; `getMeasuredShare()`), `setStrength(0..1)` (gain = 1 + (raw−1)·strength), `setMaxGain(x)` (clamp to [max(0.5, 1/x), x]). Main uses ignore = true, `WHITE_BALANCE_STRENGTH 0.5`, `WHITE_BALANCE_MAX_GAIN 1.6`; telemetry "white balance" shows the gains and the measured share. Untested. If balls are still missed with the colors looking right, the next suspect is the SDK's `ColorRange.YELLOW` itself (a custom YCrCb range may be needed).
  - **Color range fix (same day):** user photos of the DS camera stream showed the cyan contours (= matched "yellow") around the gray floor tiles, with the ball cut out as a hole. The balls are lime/greenish-yellow pickleball-style balls; the SDK's `ColorRange.YELLOW` (YCrCb min 32/128/0, max 255/170/120) needs Cr ≥ 128, so the warm-lit floor matched and the ball (Cr ≈ 125–135, Cb ≈ 65–75, estimated) did not. Main now uses its own range `BALL_MIN/MAX_Y 80–255, CR 105–145, CB 0–95` (estimates from a photo of the screen, NOT measured). New shared helper `teamcode/ColorSampler.java` (VisionProcessor, added after the white balance: `addProcessors(colorFix, colorProbe, ballLocator)`) averages a 16×16 box in the picture center, converts to YCrCb with OpenCV's formula, draws a white box on the stream; telemetry "color in the white box" shows Y/Cr/Cb and whether it is inside the range. Main's INIT is now a `while (opModeInInit())` loop that shows "camera sees" and the probe live. Team had set `BALL_MIN_AREA` 30 (from 200).
  - **Ball grab WORKS on the robot (2026-11-11)** with the custom lime-yellow range. Then, per the user: SEARCH now spins in full circles (`SEARCH_DIRECTION +1` = clockwise, power 0.25; `SEARCH_SWEEP_DEG`, `searchCenterDeg`, `searchDirection` removed), and `HIT_PUSH_SECONDS` 1.0 → 0.45 because the robot drove ~40 cm past the pickup (0.45 is an estimate, untested).
  - **Shooter gear ratio correction (2026-11-11):** the user first said 1:1, then "1:3"; asked and confirmed it is a REV UltraPlanetary **3:1 cartridge** (wheel slower than the motor). `SHOOTER_GEAR_RATIO = 84.0 / 29.0` (= 2.897, REV's real 3:1 ratio). Target 1000 wheel rpm → motor ≈ 2897 rpm → ≈ 1352 ticks/s. The F term is still computed from the bare motor (6000 rpm, 28 ticks), so it stays correct. Untested.
  - **Shooter (2026-11-11, later):** the user said the shooter ran the wrong way and asked for max power. Both directions flipped (`M2_DIRECTION REVERSE`, `M3_DIRECTION FORWARD`). New `SHOOTER_FULL_POWER = true`: after START both shooters go `RUN_WITHOUT_ENCODER` + `setPower(1.0)`, the no-encoder check is skipped, telemetry shows rpm (if encoders are connected) and "FULL POWER". The 1000 rpm velocity PID code is still there for `SHOOTER_FULL_POWER = false`. Untested.
  - **Shooter (2026-11-11, latest):** the team switched the shooter back to 1:1 (`SHOOTER_GEAR_RATIO 1.0`) and wants it to hold **1500 rpm**: `SHOOTER_RPM 1500`, `SHOOTER_FULL_POWER false` (velocity PID again; target = 1500/60×28 = 700 ticks/s, a quarter of the motor's 6000 rpm free speed). Directions stay flipped (M2 REVERSE, M3 FORWARD). Untested.
  - **Committed and pushed (2026-11-12):** all the work above is commit `023f598` on `master` in github.com/deltateamftc-glitch/Quickstart (MainTeleOp, BallDetectTeleOp, WhiteBalanceProcessor, ColorSampler, TestTeleOp, SquareButtonTest, FEATURE_LIST.md, gradle/gradle-daemon-jvm.properties). Nothing uncommitted after that. (FTC_HANDOFF.md was outside the repo then; it moved in later the same day.)
  - **New-computer setup (2026-11-12):** the handoff moved into the repo (`Quickstart/FTC_HANDOFF.md`) so a second computer (the team's laptop) gets it with `git pull`. New `Quickstart/CLAUDE.md` (auto-loaded by Claude Code) holds the team rules, robot config, versions, tooling and a step-by-step "Setting up a new computer" checklist for Claude. The desktop's `connect_robot.cmd` (adb connect 192.168.43.1:5555 over the robot Wi-Fi) is now in the repo root too, using `%LOCALAPPDATA%` instead of a fixed user folder; the setup checklist makes a desktop shortcut to it.
