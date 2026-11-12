package org.firstinspires.ftc.teamcode.experimental;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.WhiteBalanceProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// Copy of Final/MainTeleOp plus webcam ball detection (purple and green DECODE artifacts).
// HOME (PS button) cycles through three modes: NORMAL -> FOLLOW -> HIT -> NORMAL.
@TeleOp(name = "Ball Detect TeleOp", group = "Experimental")
public class BallDetectTeleOp extends LinearOpMode {

    static final Rev9AxisImuOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP;
    static final Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection I2C_PORT_DIRECTION =
            Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD;

    static final double HEADING_KP = 0.02;
    static final double HEADING_KI = 0.02;
    static final double HEADING_KD = 0.002;
    static final double HEADING_I_ZONE_DEG = 5;
    static final double HEADING_TOLERANCE_DEG = 2;
    static final double HEADING_SETTLED_DEG_PER_SEC = 10;
    static final double MAX_AUTO_TURN_POWER = 0.6;

    // Heading hold (same as Final/MainTeleOp): when neither the driver nor an automatic mode is turning,
    // keep the robot pointing the same way (uneven wheel drag would otherwise slowly rotate it while driving).
    static final boolean HEADING_HOLD_ENABLED = true;
    static final double HOLD_DEADBAND_DEG = 1;           // errors smaller than this get no angle correction
    static final double HOLD_KP = 0.02;                  // turn power per degree beyond the deadband (grows smoothly from 0)
    static final double HOLD_KD = 0.001;                 // turn power per deg/s of rotation, resists any drift right away
    static final double MAX_HOLD_TURN_POWER = 0.3;
    static final double HOLD_CAPTURE_DEG_PER_SEC = 20;   // after turning stops, start holding once slower than this

    // SHARE switches ALL stabilization on/off: field-centric (headless) driving + heading hold.
    // Off = plain robot-centric driving, e.g. if the IMU acts up. The ball modes keep working either way.
    static final boolean STABILIZATION_ON_AT_START = true;

    static final String WEBCAM_NAME = "Webcam 1";
    static final int CAMERA_WIDTH = 320;
    static final int CAMERA_HEIGHT = 240;
    static final ColorRange PURPLE_BALL_COLOR = ColorRange.ARTIFACT_PURPLE;
    static final ColorRange GREEN_BALL_COLOR = ColorRange.ARTIFACT_GREEN;
    // Blob size limits in pixels (the whole 320x240 picture is 76800 pixels).
    static final double BALL_MIN_AREA = 200;
    static final double BALL_MAX_AREA = 10000;
    // 1 = perfect circle. Lower this if a real ball is missed (shadows make it less round).
    static final double BALL_MIN_CIRCULARITY = 0.6;

    // White balance, for yellow (or blue) tinted lighting. Two separate fixes, both work in INIT and while driving:
    // 1) Software color fix (SQUARE = on/off): evens out the picture's average color automatically.
    static final boolean COLOR_FIX_ON_AT_START = true;
    // 2) The webcam's own white balance (D-pad), only if the webcam supports it. 0 = the camera decides (auto).
    //    Find a good value with DPAD UP/DOWN, then write it here so it is used every time.
    static final int CAMERA_WHITE_BALANCE_K = 0;
    static final int CAMERA_WHITE_BALANCE_STEP_K = 200;

    // Ball aiming (hold CIRCLE). Error = how many pixels the ball is right (+) or left (-) of the picture's center.
    static final double AIM_KP = 0.001;
    static final double AIM_KI = 0;
    static final double AIM_KD = 0;
    static final double AIM_I_ZONE_PX = 40;
    static final double AIM_TOLERANCE_PX = 10;   // close enough to center: stop turning
    static final double AIM_MIN_POWER = 0.08;    // smallest turn power that still moves the robot
    static final double MAX_AIM_TURN_POWER = 0.4;

    // Distance estimate: a ball looks half as big when it is twice as far away.
    // CALIBRATE: put a ball exactly CALIBRATION_DISTANCE_CM in front of the camera lens,
    // read its "r" (radius in pixels) from telemetry, and write it into CALIBRATION_RADIUS_PX.
    static final double CALIBRATION_DISTANCE_CM = 30;
    static final double CALIBRATION_RADIUS_PX = 48;   // rough guess, replace with your measurement

    // Follow mode (green balls only). Distances are measured from the camera lens.
    static final double FOLLOW_DISTANCE_CM = 10;      // how far from the ball the robot stops
    static final double FOLLOW_TOLERANCE_CM = 5;      // close enough: stop driving
    static final double FOLLOW_KP = 0.01;             // forward power per cm of distance error
    static final double FOLLOW_KI = 0;
    static final double FOLLOW_KD = 0;
    static final double FOLLOW_I_ZONE_CM = 5;
    static final double FOLLOW_MIN_POWER = 0.08;      // smallest drive power that still moves the robot
    static final double MAX_FOLLOW_DRIVE_POWER = 0.3;
    // A close ball looks big, so follow and hit modes allow bigger blobs than BALL_MAX_AREA.
    static final double FOLLOW_MAX_AREA = 76800;
    // Lost ball in follow mode: keep turning the way the ball left the picture, so a fast-rolling ball isn't lost.
    static final double FOLLOW_LOST_SECONDS = 1.5;      // give up after this long without seeing the ball
    static final double FOLLOW_LOST_MAX_TURN_DEG = 120; // ...or after turning this far
    static final double FOLLOW_LOST_TURN_POWER = 0.15;  // turn power while chasing (faster = blurrier picture)
    static final double FOLLOW_LOST_EDGE_PX = 40;       // ball vanished closer than this to the center = don't turn, just wait

    // Hit mode (green balls only): drive at the closest ball, push straight through it, then stop.
    static final double HIT_APPROACH_POWER = 0.35;     // forward power while steering toward the ball
    static final double HIT_PUSH_START_CM = 20;        // closer than this: stop steering and push straight
    static final double HIT_LOST_NEAR_CM = 40;         // ball vanished closer than this = it went under the camera, push
    static final double HIT_LOST_GRACE_SECONDS = 0.3;  // keep driving if the ball flickers out of view this briefly
    static final double HIT_PUSH_POWER = 0.6;          // was 0.4: hit harder
    static final double HIT_PUSH_SECONDS = 1.0;        // was 0.6: how long the straight push lasts (longer = drives further)

    // Search (TRIANGLE in hit mode): turn back and forth until a ball is seen.
    static final double SEARCH_TURN_POWER = 0.25;      // slow, so the camera picture isn't blurry
    static final double SEARCH_SWEEP_DEG = 90;         // turns this far left and right of where the search started

    enum Mode { NORMAL, FOLLOW, HIT }

    enum HitState {
        READY,      // just switched to hit mode: go for the first ball seen
        APPROACH,   // steering toward the ball
        PUSH,       // close: driving straight through the ball
        DONE,       // stopped, waiting for TRIANGLE
        SEARCH      // turning back and forth looking for a ball
    }

    // These live outside runOpMode so aimTurn() and followDrive() can use them.
    PIDController aimPid = new PIDController(AIM_KP, AIM_KI, AIM_KD, AIM_I_ZONE_PX);
    PIDController followPid = new PIDController(FOLLOW_KP, FOLLOW_KI, FOLLOW_KD, FOLLOW_I_ZONE_CM);
    int loopCount = 0;
    int lastAimLoop = -10;
    int lastFollowLoop = -10;

    // White balance (see updateWhiteBalance()).
    VisionPortal visionPortal;
    WhiteBalanceProcessor colorFix = new WhiteBalanceProcessor();
    boolean cameraWbChecked = false;   // asked the webcam whether it has white balance control yet
    WhiteBalanceControl cameraWb;      // null = the webcam has no white balance control
    boolean cameraWbManual = false;
    int cameraWbKelvin = 0;

    @Override
    public void runOpMode() {
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "mec0");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "mec1");
        DcMotor rearLeft   = hardwareMap.get(DcMotor.class, "mec2");
        DcMotor rearRight  = hardwareMap.get(DcMotor.class, "mec3");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        boolean imuOk = imu.initialize(new IMU.Parameters(
                new Rev9AxisImuOrientationOnRobot(LOGO_DIRECTION, I2C_PORT_DIRECTION)));
        imu.resetYaw();

        PIDController headingPid = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_I_ZONE_DEG);
        boolean turningToZero = false;
        boolean holding = false;     // heading hold is active
        double holdHeadingDeg = 0;   // the heading heading hold keeps
        boolean stabilization = STABILIZATION_ON_AT_START;

        Mode mode = Mode.NORMAL;
        HitState hitState = HitState.READY;
        String hitNote = "";
        ElapsedTime ballLostTimer = new ElapsedTime();
        double lastBallDistanceCm = 0;
        ElapsedTime pushTimer = new ElapsedTime();
        double pushHeadingDeg = 0;
        double searchCenterDeg = 0;
        int searchDirection = 1;   // +1 = turning clockwise, -1 = counterclockwise
        boolean followBallRecent = false;    // follow mode: the ball was seen recently, so chase it if it vanishes
        double followLastErrorPx = 0;        // where the ball was in the picture when last seen
        double followLastSeenDeg = 0;        // robot heading when the ball was last seen
        ElapsedTime followLostTimer = new ElapsedTime();

        // One color locator per ball color. Both look at the same camera picture.
        ColorBlobLocatorProcessor purpleLocator = buildBallLocator(PURPLE_BALL_COLOR, Color.rgb(255, 255, 0)); // yellow circles
        ColorBlobLocatorProcessor greenLocator = buildBallLocator(GREEN_BALL_COLOR, Color.rgb(255, 0, 0));      // red circles

        colorFix.setEnabled(COLOR_FIX_ON_AT_START);
        visionPortal = new VisionPortal.Builder()
                // The color fix must come first: processors run in this order on the same picture.
                .addProcessors(colorFix, purpleLocator, greenLocator)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                // Our webcam only offers 320x240 in MJPEG, not in the default YUY2 format.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .build();

        // INIT loop instead of waitForStart(), so the white balance can be adjusted before START.
        while (opModeInInit()) {
            updateWhiteBalance();
            telemetry.addData("IMU initialized", imuOk);
            telemetry.addLine("Left stick = drive / strafe, right stick = turn");
            telemetry.addLine("CROSS (X) = turn back to 0 degrees (normal mode)");
            telemetry.addLine("OPTIONS = reset heading (point the robot away from you first)");
            telemetry.addLine("Hold CIRCLE (O) = turn to center the closest ball (normal mode)");
            telemetry.addLine("HOME (PS) = next mode: NORMAL -> FOLLOW -> HIT (1/2/3 buzzes). FOLLOW and HIT: green balls only");
            telemetry.addLine("TRIANGLE = search for balls / stop searching (hit mode)");
            telemetry.addLine("SQUARE = color fix on/off, DPAD = camera white balance");
            telemetry.addLine("SHARE = stabilization (field-centric + heading hold) on/off");
            telemetry.addLine("Camera stream: DS menu (3 dots) > Camera Stream, during INIT");
            telemetry.addLine();
            addWhiteBalanceTelemetry();
            telemetry.addData("camera", visionPortal.getCameraState());
            telemetry.update();
            sleep(20);
        }

        ElapsedTime loopRateTimer = new ElapsedTime();
        int loopsSinceRateUpdate = 0;
        double loopsPerSecond = 0;

        while (opModeIsActive()) {
            loopCount++;

            // Loop speed, averaged over half a second so the number is steady enough to read.
            loopsSinceRateUpdate++;
            if (loopRateTimer.seconds() >= 0.5) {
                loopsPerSecond = loopsSinceRateUpdate / loopRateTimer.seconds();
                loopsSinceRateUpdate = 0;
                loopRateTimer.reset();
            }

            if (gamepad1.shareWasPressed()) {
                stabilization = !stabilization;
                holding = false;
                gamepad1.rumble(stabilization ? 300 : 1000);   // short buzz = on, long buzz = off
            }

            if (gamepad1.psWasPressed()) {
                mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
                turningToZero = false;
                hitState = HitState.READY;
                hitNote = "";
                followBallRecent = false;
                gamepad1.rumbleBlips(mode.ordinal() + 1);   // 1 buzz = normal, 2 = follow, 3 = hit
            }
            boolean trianglePressed = gamepad1.triangleWasPressed();   // read once per loop
            updateWhiteBalance();

            if (gamepad1.options) {
                imu.resetYaw();
                holding = false;   // the old held heading means nothing after a reset
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double headingDeg = Math.toDegrees(heading);
            double headingErrorDeg = AngleUnit.normalizeDegrees(0 - headingDeg);
            double turnRateDeg = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            if (gamepad1.crossWasPressed() && mode == Mode.NORMAL) {
                turningToZero = true;
                headingPid.reset();
            }

            // Ball detection, biggest (= closest) first. Normal mode: purple and green.
            // Follow and hit modes: green only (purple things like our sofa would fool them).
            List<Ball> balls = new ArrayList<>();
            if (mode == Mode.NORMAL) {
                addBalls(balls, "purple", purpleLocator, BALL_MAX_AREA);
                addBalls(balls, "green", greenLocator, BALL_MAX_AREA);
            } else {
                addBalls(balls, "green", greenLocator, FOLLOW_MAX_AREA);
            }
            Collections.sort(balls, (a, b) -> Integer.compare(b.area(), a.area()));
            Ball target = balls.isEmpty() ? null : balls.get(0);

            boolean driverTurning = Math.abs(gamepad1.right_stick_x) > 0.1;
            boolean driverUsingSticks = driverTurning
                    || Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.1;
            boolean settled = Math.abs(headingErrorDeg) < HEADING_TOLERANCE_DEG
                    && Math.abs(turnRateDeg) < HEADING_SETTLED_DEG_PER_SEC;
            if (driverTurning || settled || gamepad1.circle) {
                turningToZero = false;
            }

            double aimErrorPx = target == null ? 0 : target.circle().getX() - CAMERA_WIDTH / 2.0;
            double distanceCm = target == null ? 0 : target.distanceCm();
            boolean tooCloseToMeasure = target != null && target.touchesTopOrBottom();
            // 1 when the ball is centered, 0 at the picture's edge. Used to slow down until the robot faces the ball.
            double centeredFactor = Math.max(0, 1 - Math.abs(aimErrorPx) / (CAMERA_WIDTH / 2.0));

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            // Field-centric while the driver is in control. Stabilization off = robot-centric.
            double driveHeading = stabilization ? heading : 0;
            boolean chasingLostBall = false;
            boolean autoTurning = false;   // an automatic mode decides the turn power this loop

            if (mode == Mode.NORMAL) {
                // Aim at the closest ball only while CIRCLE is held, a ball is visible, and the driver isn't turning.
                autoTurning = (target != null && gamepad1.circle && !driverTurning) || turningToZero;
                if (target != null && gamepad1.circle && !driverTurning) {
                    turn = aimTurn(aimErrorPx);
                } else if (turningToZero) {
                    // Positive turn spins the robot clockwise, but heading grows counterclockwise, so flip the sign.
                    turn = Range.clip(-headingPid.update(headingErrorDeg), -MAX_AUTO_TURN_POWER, MAX_AUTO_TURN_POWER);
                }

            } else if (mode == Mode.FOLLOW) {
                // Turn toward the closest ball and keep FOLLOW_DISTANCE_CM from it. The driver's sticks win.
                if (driverUsingSticks) {
                    followBallRecent = false;   // the driver took over, so don't chase an old ball afterwards
                } else if (target == null && followBallRecent) {
                    // The ball vanished, e.g. it rolled out of the picture faster than we turned.
                    double turnedDeg = Math.abs(AngleUnit.normalizeDegrees(headingDeg - followLastSeenDeg));
                    if (followLostTimer.seconds() > FOLLOW_LOST_SECONDS || turnedDeg > FOLLOW_LOST_MAX_TURN_DEG) {
                        followBallRecent = false;   // give up and wait for a ball to show up
                    } else {
                        chasingLostBall = true;
                        autoTurning = true;
                        driveHeading = 0;
                        x = 0;
                        y = 0;
                        // Ball left on the right (+ error) -> clockwise (+ turn). Near the center -> just wait,
                        // it probably flickered out or rolled under the camera.
                        turn = Math.abs(followLastErrorPx) < FOLLOW_LOST_EDGE_PX
                                ? 0 : Math.signum(followLastErrorPx) * FOLLOW_LOST_TURN_POWER;
                    }
                } else if (target != null) {
                    followBallRecent = true;
                    followLastErrorPx = aimErrorPx;
                    followLastSeenDeg = headingDeg;
                    followLostTimer.reset();
                    autoTurning = true;
                    turn = aimTurn(aimErrorPx);
                    // Drive in the robot's own frame (where the camera looks), not field-centric.
                    driveHeading = 0;
                    x = 0;
                    double distanceErrorCm = distanceCm - FOLLOW_DISTANCE_CM;
                    if (tooCloseToMeasure || Math.abs(distanceErrorCm) < FOLLOW_TOLERANCE_CM) {
                        // A ball cut off by the picture's edge looks smaller (= farther) than it is, so never drive into it.
                        y = 0;
                    } else {
                        y = followDrive(distanceErrorCm) * centeredFactor;
                    }
                }

            } else {   // Mode.HIT
                if (driverUsingSticks && hitState != HitState.DONE) {
                    hitState = HitState.DONE;   // touching the sticks cancels the automatic driving
                    hitNote = "cancelled by driver";
                }
                if (trianglePressed) {
                    if (hitState == HitState.SEARCH) {
                        hitState = HitState.DONE;
                        hitNote = "search stopped";
                    } else {
                        hitState = HitState.SEARCH;
                        searchCenterDeg = headingDeg;
                        searchDirection = 1;
                        hitNote = "";
                    }
                }

                if (!driverUsingSticks) {
                    // Everything below drives in the robot's own frame and starts from "stand still".
                    driveHeading = 0;
                    x = 0;
                    y = 0;
                    turn = 0;

                    if ((hitState == HitState.READY || hitState == HitState.SEARCH) && target != null) {
                        hitState = HitState.APPROACH;
                        ballLostTimer.reset();
                    }

                    if (hitState == HitState.APPROACH) {
                        boolean startPush = false;
                        if (target != null) {
                            ballLostTimer.reset();
                            lastBallDistanceCm = distanceCm;
                            if (tooCloseToMeasure || distanceCm < HIT_PUSH_START_CM) {
                                startPush = true;
                            } else {
                                turn = aimTurn(aimErrorPx);
                                y = HIT_APPROACH_POWER * centeredFactor;
                            }
                        } else if (ballLostTimer.seconds() < HIT_LOST_GRACE_SECONDS) {
                            y = HIT_APPROACH_POWER;   // the ball flickered out of view, keep going straight
                        } else if (lastBallDistanceCm < HIT_LOST_NEAR_CM) {
                            startPush = true;         // it was close, so it is probably just below the camera now
                        } else {
                            hitState = HitState.DONE;
                            hitNote = "lost the ball";
                        }
                        if (startPush) {
                            hitState = HitState.PUSH;
                            pushHeadingDeg = headingDeg;
                            pushTimer.reset();
                            headingPid.reset();
                        }
                    }

                    if (hitState == HitState.PUSH) {
                        if (pushTimer.seconds() >= HIT_PUSH_SECONDS) {
                            hitState = HitState.DONE;
                            hitNote = "ball hit";
                        } else {
                            // Drive straight, holding the heading the push started with.
                            y = HIT_PUSH_POWER;
                            double pushErrorDeg = AngleUnit.normalizeDegrees(pushHeadingDeg - headingDeg);
                            turn = Range.clip(-headingPid.update(pushErrorDeg), -MAX_AUTO_TURN_POWER, MAX_AUTO_TURN_POWER);
                        }
                    }

                    if (hitState == HitState.SEARCH) {
                        // Clockwise (+ turn) makes the heading go down, counterclockwise makes it go up.
                        double offsetDeg = AngleUnit.normalizeDegrees(headingDeg - searchCenterDeg);
                        if (searchDirection > 0 && offsetDeg <= -SEARCH_SWEEP_DEG) {
                            searchDirection = -1;
                        } else if (searchDirection < 0 && offsetDeg >= SEARCH_SWEEP_DEG) {
                            searchDirection = 1;
                        }
                        turn = searchDirection * SEARCH_TURN_POWER;
                    }

                    // READY and DONE just stand still, so heading hold may keep the robot pointing the same way.
                    autoTurning = hitState == HitState.APPROACH || hitState == HitState.PUSH
                            || hitState == HitState.SEARCH;
                }
            }

            // Heading hold: if nobody is turning the robot on purpose, keep it pointing the same way.
            if (driverTurning || autoTurning || !HEADING_HOLD_ENABLED || !stabilization) {
                holding = false;
            } else if (!holding && Math.abs(turnRateDeg) < HOLD_CAPTURE_DEG_PER_SEC) {
                // The robot has (almost) stopped turning: remember this heading and keep it.
                holding = true;
                holdHeadingDeg = headingDeg;
            }
            double holdErrorDeg = holding ? AngleUnit.normalizeDegrees(holdHeadingDeg - headingDeg) : 0;
            if (holding) {
                // Only the part of the error beyond the deadband counts, so the correction starts at 0 and grows smoothly.
                double beyondDeadbandDeg = Math.max(0, Math.abs(holdErrorDeg) - HOLD_DEADBAND_DEG);
                // Positive turn spins the robot clockwise, but heading grows counterclockwise, so flip the sign.
                // The rotation term works even inside the deadband: turning counterclockwise (+ rate) gets clockwise (+) power.
                turn = -Math.signum(holdErrorDeg) * HOLD_KP * beyondDeadbandDeg + HOLD_KD * turnRateDeg;
                turn = Range.clip(turn, -MAX_HOLD_TURN_POWER, MAX_HOLD_TURN_POWER);
            }

            double stickAngle = Math.atan2(y, x);
            double theta = stickAngle - driveHeading;
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double fl = power * cos / max + turn;
            double fr = power * sin / max - turn;
            double rl = power * sin / max + turn;
            double rr = power * cos / max - turn;

            double scale = power + Math.abs(turn);
            if (scale > 1) {
                fl /= scale;
                fr /= scale;
                rl /= scale;
                rr /= scale;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            rearLeft.setPower(rl);
            rearRight.setPower(rr);

            telemetry.addData("loop speed", "%.0f /sec (%.0f ms per loop), camera %.0f pictures/sec",
                    loopsPerSecond, loopsPerSecond > 0 ? 1000 / loopsPerSecond : 0, visionPortal.getFps());
            telemetry.addData("MODE (HOME to switch)", mode);
            telemetry.addData("stabilization (SHARE)", stabilization
                    ? "ON  (field-centric + heading hold)" : "OFF (robot-centric, no heading hold)");
            if (mode == Mode.FOLLOW) {
                if (driverUsingSticks) {
                    telemetry.addData("follow", "paused, driver using sticks");
                } else if (chasingLostBall) {
                    telemetry.addData("follow", turn == 0 ? "ball lost near the center, waiting %.1f s"
                                    : "ball lost, turning " + (turn > 0 ? "right" : "left") + " to find it (%.1f s)",
                            followLostTimer.seconds());
                } else if (target == null) {
                    telemetry.addData("follow", "no ball seen, waiting");
                } else if (tooCloseToMeasure) {
                    telemetry.addData("follow", "ball too close to measure, not driving");
                } else {
                    telemetry.addData("follow", "%s ball ~%.0f cm away (target %.0f), %.0f px %s of center",
                            target.color, distanceCm, FOLLOW_DISTANCE_CM,
                            Math.abs(aimErrorPx), aimErrorPx >= 0 ? "right" : "left");
                }
            } else if (mode == Mode.HIT) {
                String status;
                switch (hitState) {
                    case READY:    status = "waiting for a ball (TRIANGLE = search)"; break;
                    case APPROACH: status = target == null ? "driving at the ball (lost from view)"
                                          : String.format("driving at %s ball ~%.0f cm", target.color, distanceCm); break;
                    case PUSH:     status = "HITTING the ball"; break;
                    case SEARCH:   status = "searching... (TRIANGLE = stop)"; break;
                    default:       status = "stopped, " + hitNote + " (TRIANGLE = search)"; break;
                }
                telemetry.addData("hit", status);
            } else if (!gamepad1.circle) {
                telemetry.addData("aim at ball (hold O)", "off");
            } else if (target == null) {
                telemetry.addData("aim at ball (hold O)", "NO BALL SEEN");
            } else if (driverTurning) {
                telemetry.addData("aim at ball (hold O)", "paused, right stick in use");
            } else {
                telemetry.addData("aim at ball (hold O)", "ACTIVE, %s ball %.0f px %s of center",
                        target.color, Math.abs(aimErrorPx), aimErrorPx >= 0 ? "right" : "left");
            }
            if (balls.isEmpty()) {
                telemetry.addLine("BALL: none seen");
            } else {
                telemetry.addLine("BALL SEEN: " + balls.size() + " ball(s), closest first");
                for (int i = 0; i < balls.size(); i++) {
                    Ball ball = balls.get(i);
                    Circle circle = ball.circle();
                    telemetry.addLine(String.format("  ball %d: %s, %s  (x %d, y %d, %d px, r %d, ~%d cm)",
                            i + 1, ball.color, segmentOf(circle.getX(), circle.getY()),
                            (int) circle.getX(), (int) circle.getY(), ball.area(),
                            (int) circle.getRadius(), (int) ball.distanceCm()));
                }
            }
            addWhiteBalanceTelemetry();
            telemetry.addData("camera", visionPortal.getCameraState());
            telemetry.addLine();

            telemetry.addData("auto turn to 0", turningToZero ? "ACTIVE" : "off");
            if (holding) {
                telemetry.addData("heading hold", "HOLDING %.1f deg, error %.1f deg", holdHeadingDeg, holdErrorDeg);
            } else {
                telemetry.addData("heading hold", !HEADING_HOLD_ENABLED || !stabilization ? "disabled"
                        : driverTurning || autoTurning ? "off while turning" : "waiting for the robot to stop turning");
            }
            telemetry.addData("robot heading (deg)", "%.1f", headingDeg);
            telemetry.addData("heading error (deg)", "%.1f", headingErrorDeg);
            telemetry.addData("turn rate (deg/s)", "%.0f", turnRateDeg);
            telemetry.addData("turn power", "%.2f", turn);
            telemetry.addLine();
            telemetry.addData("stick angle (deg)", "%.0f", Math.toDegrees(stickAngle));
            telemetry.addData("stick magnitude", "%.2f", power);
            telemetry.addLine();
            telemetry.addData("front left  (mec0)", "%.2f", fl);
            telemetry.addData("front right (mec1)", "%.2f", fr);
            telemetry.addData("rear left   (mec2)", "%.2f", rl);
            telemetry.addData("rear right  (mec3)", "%.2f", rr);
            telemetry.update();
        }
    }

    // Turn power that moves the ball to the picture's center.
    // The PID restarts fresh if it wasn't used in the previous loop, so old values don't carry over.
    double aimTurn(double aimErrorPx) {
        if (lastAimLoop != loopCount - 1) {
            aimPid.reset();
        }
        lastAimLoop = loopCount;
        if (Math.abs(aimErrorPx) < AIM_TOLERANCE_PX) {
            return 0;
        }
        // Ball right of center (+ error) needs a clockwise turn, which is + turn power, so no sign flip.
        double turn = aimPid.update(aimErrorPx) + Math.signum(aimErrorPx) * AIM_MIN_POWER;
        return Range.clip(turn, -MAX_AIM_TURN_POWER, MAX_AIM_TURN_POWER);
    }

    // SQUARE = software color fix on/off. DPAD UP/DOWN = set the webcam's white balance by hand, DPAD LEFT = back to auto.
    void updateWhiteBalance() {
        // Read every button once per loop, even while the camera isn't ready, so old presses don't pile up.
        boolean squarePressed = gamepad1.squareWasPressed();
        boolean upPressed = gamepad1.dpadUpWasPressed();
        boolean downPressed = gamepad1.dpadDownWasPressed();
        boolean leftPressed = gamepad1.dpadLeftWasPressed();

        if (squarePressed) {
            colorFix.setEnabled(!colorFix.isEnabled());
        }

        // Camera controls only work once the camera is streaming (a second or two after INIT).
        if (!cameraWbChecked) {
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                return;
            }
            cameraWbChecked = true;
            WhiteBalanceControl control = visionPortal.getCameraControl(WhiteBalanceControl.class);
            if (control != null && control.getMaxWhiteBalanceTemperature() > 0) {
                cameraWb = control;
                if (CAMERA_WHITE_BALANCE_K > 0) {
                    setCameraKelvin(CAMERA_WHITE_BALANCE_K);
                } else {
                    // The webcam remembers a manual setting from an earlier run, so switch back to auto.
                    cameraWbManual = !cameraWb.setMode(WhiteBalanceControl.Mode.AUTO);
                }
            }
        }
        if (cameraWb == null) {
            return;
        }

        if (leftPressed) {
            cameraWbManual = !cameraWb.setMode(WhiteBalanceControl.Mode.AUTO);
        }
        if (upPressed || downPressed) {
            int kelvin = cameraWbKelvin;
            if (!cameraWbManual) {
                // Start from what the camera's auto mode picked.
                kelvin = cameraWb.getWhiteBalanceTemperature();
                if (kelvin <= 0) {
                    kelvin = (cameraWb.getMinWhiteBalanceTemperature() + cameraWb.getMaxWhiteBalanceTemperature()) / 2;
                }
            }
            kelvin += upPressed ? CAMERA_WHITE_BALANCE_STEP_K : -CAMERA_WHITE_BALANCE_STEP_K;
            setCameraKelvin(kelvin);
        }
    }

    // Lower kelvin = "the light is yellow", so the camera makes the picture bluer.
    void setCameraKelvin(int kelvin) {
        kelvin = Range.clip(kelvin, cameraWb.getMinWhiteBalanceTemperature(), cameraWb.getMaxWhiteBalanceTemperature());
        cameraWbManual = cameraWb.setMode(WhiteBalanceControl.Mode.MANUAL);
        if (cameraWbManual && cameraWb.setWhiteBalanceTemperature(kelvin)) {
            cameraWbKelvin = kelvin;
        }
    }

    void addWhiteBalanceTelemetry() {
        if (colorFix.isEnabled()) {
            telemetry.addData("color fix (SQUARE)", "ON  red x%.2f  green x%.2f  blue x%.2f",
                    colorFix.getRedGain(), colorFix.getGreenGain(), colorFix.getBlueGain());
        } else {
            telemetry.addData("color fix (SQUARE)", "off");
        }
        String camera;
        if (!cameraWbChecked) {
            camera = "waiting for the camera";
        } else if (cameraWb == null) {
            camera = "not supported by this webcam (use the color fix)";
        } else if (cameraWbManual) {
            camera = String.format("MANUAL %d K  (DOWN = less yellow, UP = more yellow, LEFT = auto)", cameraWbKelvin);
        } else {
            camera = "AUTO  (DPAD UP/DOWN = set by hand)";
        }
        telemetry.addData("camera white balance", camera);
    }

    // Forward (+) or backward (-) power that brings the robot to FOLLOW_DISTANCE_CM from the ball.
    double followDrive(double distanceErrorCm) {
        if (lastFollowLoop != loopCount - 1) {
            followPid.reset();
        }
        lastFollowLoop = loopCount;
        double drive = followPid.update(distanceErrorCm) + Math.signum(distanceErrorCm) * FOLLOW_MIN_POWER;
        return Range.clip(drive, -MAX_FOLLOW_DRIVE_POWER, MAX_FOLLOW_DRIVE_POWER);
    }

    // A detected ball: its color name plus the blob the color locator found.
    static class Ball {
        final String color;
        final ColorBlobLocatorProcessor.Blob blob;

        Ball(String color, ColorBlobLocatorProcessor.Blob blob) {
            this.color = color;
            this.blob = blob;
        }

        // How many pixels the ball takes up. A bigger ball on screen is a closer ball.
        int area() {
            return blob.getContourArea();
        }

        Circle circle() {
            return blob.getCircle();
        }

        // Estimated distance from the camera lens, based on how big the ball looks (see CALIBRATION_*).
        double distanceCm() {
            double radius = Math.max(1, circle().getRadius());
            return CALIBRATION_DISTANCE_CM * CALIBRATION_RADIUS_PX / radius;
        }

        // True if the ball is cut off by the top or bottom of the picture, so its size can't be trusted.
        boolean touchesTopOrBottom() {
            for (Point p : blob.getContourPoints()) {
                if (p.y <= 2 || p.y >= CAMERA_HEIGHT - 3) {
                    return true;
                }
            }
            return false;
        }
    }

    static ColorBlobLocatorProcessor buildBallLocator(ColorRange color, int circleDrawColor) {
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)                 // outline matching color areas on the camera stream
                .setBoxFitColor(0)                     // no rectangles
                .setCircleFitColor(circleDrawColor)    // circle around each ball
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
    }

    // Keeps only round blobs of a sensible size from one locator and adds them to the list.
    static void addBalls(List<Ball> balls, String color, ColorBlobLocatorProcessor locator, double maxArea) {
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, BALL_MIN_AREA, maxArea, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, BALL_MIN_CIRCULARITY, 1, blobs);
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            balls.add(new Ball(color, blob));
        }
    }

    // Splits the camera picture into a 3x3 grid and names the cell the point is in,
    // e.g. "bottom left". Pixel (0,0) is the top-left corner of the picture.
    static String segmentOf(double x, double y) {
        String column;
        if (x < CAMERA_WIDTH / 3.0) {
            column = "left";
        } else if (x < CAMERA_WIDTH * 2 / 3.0) {
            column = "center";
        } else {
            column = "right";
        }

        String row;
        if (y < CAMERA_HEIGHT / 3.0) {
            row = "top";
        } else if (y < CAMERA_HEIGHT * 2 / 3.0) {
            row = "middle";
        } else {
            row = "bottom";
        }

        if (row.equals("middle") && column.equals("center")) {
            return "center";
        }
        return row + " " + column;
    }
}
