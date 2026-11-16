# Feature List — deltateamftc

All the features we have built, with a short description of each.
Where it lives: **Main** = Main TeleOp (`Final/MainTeleOp.java`, the competition OpMode, only holds what the robot needs in a match), **Ball** = Ball Detect TeleOp (`experimental/BallDetectTeleOp.java`), **Test** = Test TeleOp (`experimental/TestTeleOp.java`, the test bench: new ideas are tried there first).
"(untested)" = compiles, but not yet tried on the robot.

## Driving

- **Mecanum drive** (Main, Ball, Test): Moves in any direction and turns at the same time. Uses the sin/cos (polar) formula: left stick = drive/strafe, right stick = turn.
- **Headless / field-centric mode** (Main, Ball, Test): "Forward" on the stick is always away from the driver, no matter which way the robot faces. Uses the IMU heading. In Main it is always on.
- **Heading reset** (Main, Ball, Test): Hold OPTIONS to make the current direction the new "forward" (0°). Point the robot away from you first.
- **Auto-turn to 0°** (Ball, Test): Press CROSS (✕) and the robot turns back to 0° by itself with a PID controller. It only stops once it is really still, so it doesn't overshoot. The right stick cancels it. (Not in Main.)
- **Heading hold** (Main, Ball, Test) (untested): When nobody is turning the robot, it keeps pointing the same way, so uneven wheel drag or a bump can't rotate it. It resists drift right away and adds a smooth correction past 1°, with no sudden jerks. In Main it is always on.
- **Point turning** (Main) (untested): TRIANGLE switches the right stick between NORMAL turning (left / right = turn) and POINT turning: the robot turns to face the way the right stick points, seen from the driver (up = away from you, right = to your right, down = toward you, any angle in between), no matter which way it faces now. Push the stick at least halfway (`POINT_STICK_MIN`). When you let go, the robot finishes the turn and heading hold keeps that direction. Short buzz = POINT, long buzz = NORMAL. Starts in NORMAL (`POINT_TURN_AT_START`). Turn speed: `POINT_KP/KI/KD` (starting values = TestTeleOp's tuned auto-turn).
- **Stabilization on/off switch** (Ball, Test) (untested): SHARE switches headless mode and heading hold off together, for plain robot-centric driving. Press again to switch them back on. Short buzz = on, long buzz = off. (Not in Main.)

## Intake and shooting (Main)

- **Intake** (Main) (untested): Tap DPAD DOWN = main intake motor "m0" spins forward at 0.7 power and keeps spinning (the second wheel "m1" stays still). Tap DPAD DOWN again = stop. Power is `INTAKE_POWER`.
- **Shooting sequence** (Main) (untested): Tap DPAD UP = second intake wheel "m1" spins at 0.7 power (`M1_POWER`) the whole time, while "m0" first pushes inward for 0.25 s (`HICCUP_START_FORWARD_SECONDS`, so the first ball cannot slip out), then keeps doing "hiccups" for the whole sequence: backwards for 0.1 s at 0.7 power, then inward for 0.5 s at 0.7 power (`HICCUP_FORWARD_POWER`), over and over, so every ball reaches the shooter. Tap DPAD UP again = the sequence is over (m0 and m1 stop). DPAD DOWN does nothing while shooting.
- **Shooter** (Main) (untested): One motor "m2" (REV UltraPlanetary with encoder; one-shooter design since 2026-11-16, "m3" was removed) holds a constant 3000 rpm (`SHOOTER_RPM`) from START to the end of the match. Its direction `M2_DIRECTION` is REVERSE, kept from the old two-motor shooter; check it. `SHOOTER_FULL_POWER = true` runs it at plain full power instead. The hub's own velocity controller does it, with an aggressive PIDF (`SHOOTER_P/I/D`, starting values, not tuned) so the wheels get back to speed fast after each ball. Telemetry shows the real rpm and "AT SPEED". Shooter gear ratio is 1:1 again (`SHOOTER_GEAR_RATIO 1.0`; the team removed the 3:1 cartridge). If the shooter reports no speed (encoder missing), it runs on plain power instead and telemetry warns.

## Servo (Main)

- **Servo s0 switch** (Main) (untested): CROSS (✕) moves the goBILDA Torque servo "s0" (Expansion Hub servo port 0) to the other of its two positions: `SERVO_POSITION_1` = 0 and `SERVO_POSITION_2` = 1 (to be tuned). When START is pressed it goes to the first position (0); it never moves during INIT.

## Ball grab (webcam)

- **Ball grab** (Main) (untested): Press HOME (PS) and the robot searches by spinning around in full circles (`SEARCH_DIRECTION`). When it sees yellow balls it stops for 0.2 s to make sure (`LOCK_CONFIRM_SECONDS`), then drives at the biggest GROUP of balls (`CLUSTER_DISTANCE_PX`), pushes through for 0.45 s (`HIT_PUSH_SECONDS`; 1 s drove ~40 cm too far), and starts searching again, over and over. The intake (m0) runs by itself the whole time the ball grab is on (`AUTO_INTAKE_DURING_GRAB`), unless a shooting sequence is running; afterwards it goes back to what DPAD DOWN had set. HOME again, or touching a stick, stops it. Telemetry "camera sees" shows how many yellow blobs the camera finds, so you can tell a real ball from a false lock.
- **Lost-ball chase** (Main) (untested in Main): If the ball leaves the picture while it is still far away (overshoot), the robot turns the way the ball left, for up to 1.5 s / 120°, until it sees it again. If the ball vanishes close up, it is assumed to be under the camera and the robot pushes.
- **Ball detection** (Main, Ball): Finds lime-yellow balls in Main (new season; our own YCrCb range `BALL_MIN/MAX_Y/CR/CB`, because the SDK's YELLOW caught the gray floor and missed the greenish-yellow ball; untested) and green balls in Ball (purple too) in the webcam picture by color, keeps only round blobs of a sensible size, and takes the biggest (= closest) one. In Ball the telemetry also shows where each one is (3×3 grid, e.g. "bottom left").
- **Color probe** (Main) (untested): A white box in the middle of the camera stream; telemetry "color in the white box" shows its Y / Cr / Cb and says "BALL COLOR" if that is inside the ball range. Put a ball in the box to tune the range. Works live during INIT.
- **Distance estimate** (Main, Ball): Guesses how far a ball is from how big it looks. Not calibrated yet (`CALIBRATION_RADIUS_PX`).
- **Ball aiming** (Ball): Hold CIRCLE (NORMAL mode) and the robot turns to face the closest ball.
- **Modes: NORMAL / FOLLOW / HIT** (Ball): HOME (PS) switches mode, with 1 / 2 / 3 rumble buzzes. FOLLOW and HIT only chase green balls (our purple sofa fooled them).
- **Follow mode** (Ball): The robot turns toward the green ball and drives forward or back to stay about 10 cm from it. Touching a stick pauses it.
- **Color fix / auto white balance** (Main, Ball) (untested in Main): Corrects yellow-tinted lighting in software so the ball colors still match. SQUARE = on/off. In Main it is always on (no button), but gentler: it only measures the light on grayish pixels (yellow balls are left out) and applies half the correction (`WHITE_BALANCE_STRENGTH 0.5`, `WHITE_BALANCE_MAX_GAIN 1.6`), because the full correction turned our yellow room blue and the balls were no longer found.
- **Camera white balance** (Ball) (untested): The webcam's own white balance setting. DPAD UP/DOWN = adjust by hand, DPAD LEFT = back to auto (only if the webcam supports it). (Not in Main.)

## Test bench (Test TeleOp)

- **Intake tests** (Test) (untested): The old intake experiments, moved here from Main. DPAD DOWN / UP = m0 one way / the other (tap = keeps spinning, tap again = stop, tap the other = reverse), DPAD RIGHT / LEFT = the same for m1, TRIANGLE (Y) = m0 "unjam" move (0.08 s one way, 0.5 s the other), plus a live current (amps) readout of m0 with the peak since INIT.

## Info on the Driver Station

- **Telemetry** (Main): Short on purpose: intake state, shooter rpm, ball grab state, heading.
- **Telemetry** (Ball, Test): Shows heading, heading error, turn rate, the four wheel powers, mode states and ball info.
- **Loop speed display** (Ball): Shows how many loops per second the code runs and how many pictures per second the camera delivers.

## Test and helper code

- **PID controller** (`PIDController.java`): Shared helper used for ball aiming, the push heading and auto-turn. Has anti-windup, so the integral can't build up while the error is big.
- **White balance processor** (`WhiteBalanceProcessor.java`): Shared helper behind the color fix (used by Ball only).
- **Motor ID Test** (experimental): Spins one motor at a time to find which port drives which wheel.
- **IMU Acceleration Test** (experimental): Reads acceleration straight from the IMU chip (just for testing, it can't be used for position).
- **Robot-centric Mecanum TeleOp** (experimental): Simple drive without the IMU.
- **Circle Button Test** (experimental): Our first hello-world OpMode.
- **Square Button Test** (experimental) (untested): While you hold SQUARE, telemetry says "Square is pressed". Nothing else.

## Setup and tools

- **Connect robot script** (`connect_robot.cmd`): After joining the robot's Wi-Fi, double-click it (or the "Connect robot" desktop shortcut) so Android Studio finds the Control Hub.
- **Claude notes** (`CLAUDE.md`): Instructions Claude Code reads automatically: our rules, robot config, and a step-by-step checklist to set the whole project up on a new computer.

## Installed but not set up yet

- **Pedro Pathing 3.0.1**: Path-following library for autonomous. It needs odometry hardware first (we have none yet).
