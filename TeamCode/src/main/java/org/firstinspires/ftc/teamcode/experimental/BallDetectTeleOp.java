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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;
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

        Mode mode = Mode.NORMAL;
        HitState hitState = HitState.READY;
        String hitNote = "";
        ElapsedTime ballLostTimer = new ElapsedTime();
        double lastBallDistanceCm = 0;
        ElapsedTime pushTimer = new ElapsedTime();
        double pushHeadingDeg = 0;
        double searchCenterDeg = 0;
        int searchDirection = 1;   // +1 = turning clockwise, -1 = counterclockwise

        // One color locator per ball color. Both look at the same camera picture.
        ColorBlobLocatorProcessor purpleLocator = buildBallLocator(PURPLE_BALL_COLOR, Color.rgb(255, 255, 0)); // yellow circles
        ColorBlobLocatorProcessor greenLocator = buildBallLocator(GREEN_BALL_COLOR, Color.rgb(255, 0, 0));      // red circles

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessors(purpleLocator, greenLocator)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                // Our webcam only offers 320x240 in MJPEG, not in the default YUY2 format.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .build();

        telemetry.addData("IMU initialized", imuOk);
        telemetry.addLine("Left stick = drive / strafe, right stick = turn");
        telemetry.addLine("CROSS (X) = turn back to 0 degrees (normal mode)");
        telemetry.addLine("OPTIONS = reset heading (point the robot away from you first)");
        telemetry.addLine("Hold CIRCLE (O) = turn to center the closest ball (normal mode)");
        telemetry.addLine("HOME (PS) = next mode: NORMAL -> FOLLOW -> HIT (1/2/3 buzzes). FOLLOW and HIT: green balls only");
        telemetry.addLine("TRIANGLE = search for balls / stop searching (hit mode)");
        telemetry.addLine("Camera stream: DS menu (3 dots) > Camera Stream, during INIT");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            loopCount++;

            if (gamepad1.psWasPressed()) {
                mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
                turningToZero = false;
                hitState = HitState.READY;
                hitNote = "";
                gamepad1.rumbleBlips(mode.ordinal() + 1);   // 1 buzz = normal, 2 = follow, 3 = hit
            }
            boolean trianglePressed = gamepad1.triangleWasPressed();   // read once per loop

            if (gamepad1.options) {
                imu.resetYaw();
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
            double driveHeading = heading;   // field-centric while the driver is in control

            if (mode == Mode.NORMAL) {
                // Aim at the closest ball only while CIRCLE is held, a ball is visible, and the driver isn't turning.
                if (target != null && gamepad1.circle && !driverTurning) {
                    turn = aimTurn(aimErrorPx);
                } else if (turningToZero) {
                    // Positive turn spins the robot clockwise, but heading grows counterclockwise, so flip the sign.
                    turn = Range.clip(-headingPid.update(headingErrorDeg), -MAX_AUTO_TURN_POWER, MAX_AUTO_TURN_POWER);
                }

            } else if (mode == Mode.FOLLOW) {
                // Turn toward the closest ball and keep FOLLOW_DISTANCE_CM from it. The driver's sticks win.
                if (target != null && !driverUsingSticks) {
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
                }
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

            telemetry.addData("MODE (HOME to switch)", mode);
            if (mode == Mode.FOLLOW) {
                if (driverUsingSticks) {
                    telemetry.addData("follow", "paused, driver using sticks");
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
            telemetry.addData("camera", visionPortal.getCameraState());
            telemetry.addLine();

            telemetry.addData("auto turn to 0", turningToZero ? "ACTIVE" : "off");
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
