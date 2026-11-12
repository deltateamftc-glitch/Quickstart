package org.firstinspires.ftc.teamcode.Final;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.ColorSampler;
import org.firstinspires.ftc.teamcode.WhiteBalanceProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

// Competition TeleOp. It only has what the robot needs in a match:
//   1) Driving: left stick = drive / strafe, right stick = turn. Always field-centric (headless) with heading hold.
//   2) Ball grab: HOME (PS) = search for yellow balls, lock onto the biggest group, drive at it and push through it,
//      then search again, over and over. HOME again stops it. A software white balance
//      (color fix for tinted light) is always on, so yellow light doesn't fool the color detection.
//   3) Intake / shooting sequence on the D-pad (m0 = main intake, m1 = second intake wheel).
//   4) Shooter wheels (m2, m3) hold a constant speed from START until the end of the match.
// New or untested ideas go to experimental/TestTeleOp first.
@TeleOp(name = "Main TeleOp", group = "Final")
public class MainTeleOp extends LinearOpMode {

    // ---------------------------------------------------------------- driving
    static final Rev9AxisImuOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP;
    static final Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection I2C_PORT_DIRECTION =
            Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD;

    // Heading hold: when nobody is turning the robot on purpose, keep it pointing the same way
    // (uneven wheel drag or a bump would otherwise rotate it).
    static final boolean HEADING_HOLD_ENABLED = true;
    static final double HOLD_DEADBAND_DEG = 1;           // errors smaller than this get no angle correction
    static final double HOLD_KP = 0.02;                  // turn power per degree beyond the deadband (grows smoothly from 0)
    static final double HOLD_KD = 0.001;                 // turn power per deg/s of rotation, resists any drift right away
    static final double MAX_HOLD_TURN_POWER = 0.3;
    static final double HOLD_CAPTURE_DEG_PER_SEC = 20;   // after turning stops, start holding once slower than this

    // ---------------------------------------------------------------- intake and shooting sequence
    // DPAD DOWN = main intake (m0) spins forward until DPAD DOWN is pressed again (second wheel m1 stays still).
    // DPAD UP   = shooting sequence: m1 spins the whole time, and m0 keeps doing "hiccups" (backwards
    //             HICCUP_BACK_SECONDS, then forward HICCUP_FORWARD_SECONDS, over and over) so every ball reaches the shooter.
    //             The sequence is over when DPAD UP is pressed again (that stops m0 and m1).
    static final double INTAKE_POWER = 0.7;              // m0, forward (+) = picking balls up
    static final double M1_POWER = 0.7;                  // m1 during shooting
    // If a wheel spins the wrong way, flip its direction here (FORWARD <-> REVERSE). Nothing else needs to change.
    static final DcMotorSimple.Direction M0_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction M1_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final double HICCUP_START_FORWARD_SECONDS = 0.25;   // m0 pushes inward this long BEFORE the first hiccup (so no ball slips out at the start)
    static final double HICCUP_BACK_SECONDS = 0.1;
    static final double HICCUP_FORWARD_SECONDS = 0.5;
    static final double HICCUP_START_POWER = 0.7;        // power of the first inward push
    static final double HICCUP_BACK_POWER = 0.7;         // power of the short backwards part of each hiccup
    static final double HICCUP_FORWARD_POWER = 0.7;      // power of the long inward part of each hiccup

    // ---------------------------------------------------------------- shooter wheels (m2, m3)
    // REV UltraPlanetary motors with encoders. The hub's own velocity controller (PIDF) holds the speed.
    // true = the shooter wheels just run at full power (1.0) from START, no speed control.
    // false = they hold SHOOTER_RPM with the PID below.
    static final boolean SHOOTER_FULL_POWER = false;
    static final double SHOOTER_RPM = 3000;              // wheel speed, constantly from START (only if SHOOTER_FULL_POWER is false)
    // Gear ratio = motor turns per shooter wheel turn. 1.0 = no gearbox cartridge (the team went back to 1:1).
    // REV UltraPlanetary cartridges: 3:1 = 2.89 (84/29), 4:1 = 3.61, 5:1 = 5.23.
    static final double SHOOTER_GEAR_RATIO = 1.0;
    static final double MOTOR_ENCODER_TICKS_PER_REV = 28;  // REV UltraPlanetary HD Hex motor encoder, at the motor shaft
    static final double MOTOR_MAX_RPM = 6000;              // free speed of that motor, used for the feedforward (F)
    // Aggressive on purpose: every ball slows the wheels down and they must get back to speed fast.
    // Starting values, tune them while watching the rpm on the Driver Station (P/I up = faster recovery, too high = shaking).
    static final double SHOOTER_P = 100;
    static final double SHOOTER_I = 5;
    static final double SHOOTER_D = 0;
    // The two wheels usually face each other, so one motor must turn the other way. Flip one if a wheel spins the wrong way.
    // Both flipped on 2026-11-11 (the team said the shooter ran the wrong way).
    static final DcMotorSimple.Direction M2_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction M3_DIRECTION = DcMotorSimple.Direction.FORWARD;
    // Safety: if a shooter motor reports (almost) no speed this long after START, its encoder cable is probably
    // missing. Then it runs on plain power instead (the speed controller would otherwise go to full power).
    static final double SHOOTER_ENCODER_CHECK_SECONDS = 1.0;
    static final double SHOOTER_READY_TOLERANCE = 0.03;  // within 3% of the target counts as "at speed"

    // ---------------------------------------------------------------- ball grab (camera)
    static final String WEBCAM_NAME = "Webcam 1";
    static final int CAMERA_WIDTH = 320;
    static final int CAMERA_HEIGHT = 240;
    // New season: the game pieces are lime-yellow (greenish yellow) balls. The SDK's ColorRange.YELLOW did NOT work:
    // it wants orange-ish yellow (Cr 128 and up), so it caught our gray floor in the yellow light and missed the ball.
    // Our own range, in YCrCb: Y = brightness, Cr = how red (128 = neutral), Cb = how blue (128 = neutral).
    // The ball has very little blue (low Cb), the floor and walls are near neutral (Cb ~120), wood is too red (Cr ~150+).
    // TUNE IT: put the ball in the white box in the middle of the camera stream, read "color in the white box" on the
    // telemetry, and keep that ball's Y / Cr / Cb comfortably inside these limits.
    static final double BALL_MIN_Y = 80;
    static final double BALL_MAX_Y = 255;
    static final double BALL_MIN_CR = 105;
    static final double BALL_MAX_CR = 145;
    static final double BALL_MIN_CB = 0;
    static final double BALL_MAX_CB = 95;
    static final ColorRange BALL_COLOR = new ColorRange(ColorSpace.YCrCb,
            new Scalar(BALL_MIN_Y, BALL_MIN_CR, BALL_MIN_CB),
            new Scalar(BALL_MAX_Y, BALL_MAX_CR, BALL_MAX_CB));
    // Blob size limits in pixels (the whole 320x240 picture is 76800 pixels). A close ball looks big.
    static final double BALL_MIN_AREA = 30;
    static final double BALL_MAX_AREA = 30000;   // anything bigger (a wall, the floor, a lamp) is not a ball
    // 1 = perfect circle. Lower this if a real ball is missed (shadows make it less round).
    static final double BALL_MIN_CIRCULARITY = 0.6;
    // White balance (always on). The full correction turned the picture blue in our yellow room, so the yellow balls
    // were not yellow anymore. Now it only measures the light on grayish things (yellow balls are left out) and only
    // applies part of the correction. Picture still too blue: lower the strength. Still too yellow: raise it.
    static final double WHITE_BALANCE_STRENGTH = 0.5;   // 0 = no correction, 1 = full
    static final double WHITE_BALANCE_MAX_GAIN = 1.6;   // the most one color may be boosted
    // A ball must stay in view this long (robot standing still) before the robot locks onto it and drives.
    // Stops one flickering blob from sending the robot off.
    static final double LOCK_CONFIRM_SECONDS = 0.2;
    // With several balls in view the robot drives toward the biggest group: balls whose centers are
    // closer than this (pixels) count as one group. The group with the most ball area wins.
    static final double CLUSTER_DISTANCE_PX = 100;
    // After a hit (or after losing the ball) the robot searches again, until HOME is pressed.
    static final boolean SEARCH_AGAIN_AFTER_HIT = true;
    // The intake (m0) runs by itself while the ball grab is active (searching, driving, pushing).
    static final boolean AUTO_INTAKE_DURING_GRAB = true;

    // Aiming. Error = how many pixels the ball is right (+) or left (-) of the picture's center.
    static final double AIM_KP = 0.001;
    static final double AIM_KI = 0;
    static final double AIM_KD = 0;
    static final double AIM_I_ZONE_PX = 40;
    static final double AIM_TOLERANCE_PX = 10;   // close enough to center: stop turning
    static final double AIM_MIN_POWER = 0.08;    // smallest turn power that still moves the robot
    static final double MAX_AIM_TURN_POWER = 0.4;

    // Distance estimate: a ball looks half as big when it is twice as far away.
    // CALIBRATE: put a ball exactly CALIBRATION_DISTANCE_CM in front of the camera lens,
    // read its radius in pixels, and write it into CALIBRATION_RADIUS_PX.
    static final double CALIBRATION_DISTANCE_CM = 30;
    static final double CALIBRATION_RADIUS_PX = 48;   // rough guess, never measured

    // Drive at the ball and push through it, then stop.
    static final double HIT_APPROACH_POWER = 0.35;     // forward power while steering toward the ball
    static final double HIT_PUSH_START_CM = 20;        // closer than this: stop steering and push straight
    static final double HIT_LOST_NEAR_CM = 40;         // ball vanished closer than this = it went under the camera, push
    static final double HIT_LOST_GRACE_SECONDS = 0.3;  // keep driving if the ball flickers out of view this briefly
    static final double HIT_PUSH_POWER = 0.6;
    // How long the straight push lasts (longer = drives further). 1.0 s drove ~40 cm past the pickup, so it is
    // much shorter now. Still too far: lower it. Ball not picked up any more: raise it a little (0.05 s steps).
    static final double HIT_PUSH_SECONDS = 0.45;
    // The push holds the heading it started with.
    static final double PUSH_HEADING_KP = 0.02;
    static final double PUSH_HEADING_KI = 0.02;
    static final double PUSH_HEADING_KD = 0.002;
    static final double PUSH_HEADING_I_ZONE_DEG = 5;
    static final double MAX_PUSH_TURN_POWER = 0.6;

    // Lost-ball chase ("overshoot search"): the ball left the picture while it was still far away, e.g. it rolled out
    // faster than we turned. Keep turning the way it left, so it comes back into view.
    static final double LOST_SECONDS = 1.5;            // give up after this long without seeing the ball
    static final double LOST_MAX_TURN_DEG = 120;       // ...or after turning this far
    static final double LOST_TURN_POWER = 0.15;        // slow, so the picture isn't blurry
    static final double LOST_EDGE_PX = 40;             // vanished closer than this to the center = don't turn, just wait

    // Search (HOME with no ball in view): turn back and forth until a ball is seen.
    static final double SEARCH_TURN_POWER = 0.25;      // slow, so the camera picture isn't blurry
    static final int SEARCH_DIRECTION = 1;             // the robot spins in full circles: +1 = clockwise, -1 = counterclockwise

    enum IntakeState {
        OFF,
        COLLECTING,   // m0 forward
        SHOOTING      // m1 spinning, m0 doing its hiccups
    }

    enum HitState {
        IDLE,       // not doing anything, the driver drives
        SEARCH,     // turning back and forth looking for a ball
        APPROACH,   // steering toward the ball
        LOST,       // the ball left the picture while far away: turning to find it again
        PUSH        // close: driving straight through the ball
    }

    // These live outside runOpMode so aimTurn() can use them.
    PIDController aimPid = new PIDController(AIM_KP, AIM_KI, AIM_KD, AIM_I_ZONE_PX);
    int loopCount = 0;
    int lastAimLoop = -10;
    // What the camera saw this loop, for the telemetry (set by findTarget).
    int rawBlobCount = 0;
    double rawBiggestArea = 0;
    double rawBiggestCircularity = 0;

    @Override
    public void runOpMode() {
        // Read all hub values (encoders, velocities) in one go per loop: faster loops.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // ---- drive motors
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

        // ---- intake motors
        DcMotor intake = hardwareMap.get(DcMotor.class, "m0");    // main intake
        DcMotor intake2 = hardwareMap.get(DcMotor.class, "m1");   // second intake wheel
        intake.setDirection(M0_DIRECTION);
        intake2.setDirection(M1_DIRECTION);

        // ---- shooter motors
        DcMotorEx[] shooters = {hardwareMap.get(DcMotorEx.class, "m2"), hardwareMap.get(DcMotorEx.class, "m3")};
        String[] shooterNames = {"m2", "m3"};
        shooters[0].setDirection(M2_DIRECTION);
        shooters[1].setDirection(M3_DIRECTION);
        double shooterTicksPerRev = MOTOR_ENCODER_TICKS_PER_REV * SHOOTER_GEAR_RATIO;   // per wheel turn
        double shooterTargetTps = SHOOTER_RPM / 60.0 * shooterTicksPerRev;              // ticks per second
        // F = how much "push" a given speed needs by itself, so the controller only has to fix the small difference.
        double shooterF = 32767.0 / (MOTOR_MAX_RPM / 60.0 * MOTOR_ENCODER_TICKS_PER_REV);
        boolean[] shooterNoEncoder = {false, false};
        for (DcMotorEx shooter : shooters) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, shooterF);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // ---- IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        boolean imuOk = imu.initialize(new IMU.Parameters(
                new Rev9AxisImuOrientationOnRobot(LOGO_DIRECTION, I2C_PORT_DIRECTION)));
        imu.resetYaw();

        // ---- camera
        ColorBlobLocatorProcessor ballLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(BALL_COLOR)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)                 // outline matching color areas on the camera stream
                .setBoxFitColor(0)                     // no rectangles
                .setCircleFitColor(Color.rgb(255, 0, 0))   // red circle around each ball
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        // Measures the color in the white box in the middle of the stream (for tuning BALL_COLOR).
        ColorSampler colorProbe = new ColorSampler();
        WhiteBalanceProcessor colorFix = new WhiteBalanceProcessor();
        colorFix.setEnabled(true);   // always on
        colorFix.setIgnoreColorfulPixels(true);
        colorFix.setStrength(WHITE_BALANCE_STRENGTH);
        colorFix.setMaxGain(WHITE_BALANCE_MAX_GAIN);
        VisionPortal visionPortal = new VisionPortal.Builder()
                // The color fix must come first: processors run in this order on the same picture.
                .addProcessors(colorFix, colorProbe, ballLocator)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                // Our webcam only offers 320x240 in MJPEG, not in the default YUY2 format.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .build();

        // ---- state
        PIDController pushPid = new PIDController(PUSH_HEADING_KP, PUSH_HEADING_KI, PUSH_HEADING_KD, PUSH_HEADING_I_ZONE_DEG);
        boolean holding = false;     // heading hold is active
        double holdHeadingDeg = 0;   // the heading heading hold keeps

        IntakeState intakeState = IntakeState.OFF;
        ElapsedTime hiccupTimer = new ElapsedTime();   // time since the shooting sequence began

        HitState hitState = HitState.IDLE;
        String hitNote = "";
        ElapsedTime ballLostTimer = new ElapsedTime();   // time since the ball was last seen
        double lastBallDistanceCm = 0;
        double lastBallErrorPx = 0;      // where the ball was in the picture when last seen
        double lastSeenHeadingDeg = 0;   // robot heading when the ball was last seen
        ElapsedTime pushTimer = new ElapsedTime();
        double pushHeadingDeg = 0;
        ElapsedTime seenTimer = new ElapsedTime();   // how long the ball has been in view without a gap
        int hitsDone = 0;

        // INIT loop instead of a plain waitForStart(), so the camera numbers update live while you look at the stream.
        // Nothing moves here.
        while (opModeInInit()) {
            telemetry.addData("IMU initialized", imuOk);
            telemetry.addLine("Left stick = drive / strafe, right stick = turn (field-centric + heading hold)");
            telemetry.addLine("OPTIONS = reset heading (point the robot away from you first)");
            telemetry.addLine("HOME (PS) = search for yellow balls, hit them one after another / HOME again = stop. Moving a stick cancels it");
            telemetry.addLine("DPAD DOWN = intake on / off.  DPAD UP = shooting sequence start / end");
            telemetry.addLine("Shooter wheels m2/m3 start at "
                    + (SHOOTER_FULL_POWER ? "FULL POWER" : (int) SHOOTER_RPM + " rpm") + " when you press START");
            telemetry.addLine("Camera stream: DS menu (3 dots) > Camera Stream, during INIT (tap the picture to refresh)");
            telemetry.addLine();
            Target initTarget = findTarget(ballLocator);
            telemetry.addData("camera sees", String.format("%d yellow blob(s), biggest %.0f px, roundness %.2f -> %s",
                    rawBlobCount, rawBiggestArea, rawBiggestCircularity,
                    initTarget == null ? "no ball" : initTarget.count + " ball(s) in the best group"));
            addColorProbeTelemetry(colorProbe);
            telemetry.update();
            sleep(50);
        }
        ElapsedTime shooterTimer = new ElapsedTime();
        for (DcMotorEx shooter : shooters) {
            if (SHOOTER_FULL_POWER) {
                shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // no speed control, just full power
                shooter.setPower(1.0);
            } else {
                shooter.setVelocity(shooterTargetTps);   // the hub keeps this speed from now on
            }
        }

        while (opModeIsActive()) {
            loopCount++;

            boolean homePressed = gamepad1.psWasPressed();   // HOME / PS button. Read each button once per loop
            boolean downTapped = gamepad1.dpadDownWasPressed();
            boolean upTapped = gamepad1.dpadUpWasPressed();

            if (gamepad1.options) {
                imu.resetYaw();
                holding = false;   // the old held heading means nothing after a reset
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double headingDeg = Math.toDegrees(heading);
            double turnRateDeg = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            // ------------------------------------------------ intake / shooting sequence
            if (upTapped) {
                if (intakeState == IntakeState.SHOOTING) {
                    intakeState = IntakeState.OFF;          // shooting sequence over
                } else {
                    intakeState = IntakeState.SHOOTING;     // (also works while collecting: switches over)
                    hiccupTimer.reset();   // the first hiccup starts now
                }
            } else if (downTapped) {
                if (intakeState == IntakeState.COLLECTING) {
                    intakeState = IntakeState.OFF;
                } else if (intakeState == IntakeState.OFF) {
                    intakeState = IntakeState.COLLECTING;
                }
                // While shooting, DPAD DOWN does nothing: end the sequence with DPAD UP first.
            }

            double m0Power = 0;
            double m1Power = 0;
            // The ball grab (HOME) runs the intake by itself the whole time it is active, unless we are shooting.
            // When the grab ends, the intake goes back to what DPAD DOWN had set.
            boolean grabIntake = AUTO_INTAKE_DURING_GRAB && hitState != HitState.IDLE;
            if (intakeState == IntakeState.COLLECTING || (grabIntake && intakeState == IntakeState.OFF)) {
                m0Power = INTAKE_POWER;
            } else if (intakeState == IntakeState.SHOOTING) {
                m1Power = M1_POWER;
                // First a short push inward (so the first ball can't slip out), then hiccups over and over
                // (back, then forward, back, then forward...) until the sequence is ended.
                double elapsed = hiccupTimer.seconds();
                if (elapsed < HICCUP_START_FORWARD_SECONDS) {
                    m0Power = HICCUP_START_POWER;
                } else {
                    double t = (elapsed - HICCUP_START_FORWARD_SECONDS) % (HICCUP_BACK_SECONDS + HICCUP_FORWARD_SECONDS);
                    m0Power = t < HICCUP_BACK_SECONDS ? -HICCUP_BACK_POWER : HICCUP_FORWARD_POWER;
                }
            }
            intake.setPower(m0Power);
            intake2.setPower(m1Power);

            // ------------------------------------------------ shooter wheels
            double[] shooterRpm = new double[2];
            boolean shooterAtSpeed = true;
            for (int i = 0; i < shooters.length; i++) {
                double tps = shooters[i].getVelocity();
                shooterRpm[i] = tps / shooterTicksPerRev * 60.0;
                if (!SHOOTER_FULL_POWER && !shooterNoEncoder[i] && shooterTimer.seconds() > SHOOTER_ENCODER_CHECK_SECONDS
                        && Math.abs(tps) < 0.1 * shooterTargetTps) {
                    // No speed reading: no encoder. Run on plain power instead of letting the controller floor it.
                    shooterNoEncoder[i] = true;
                    shooters[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double maxTps = MOTOR_MAX_RPM / 60.0 * MOTOR_ENCODER_TICKS_PER_REV;
                    shooters[i].setPower(Range.clip(shooterTargetTps / maxTps, 0, 1));
                }
                if (shooterNoEncoder[i]
                        || Math.abs(shooterRpm[i] - SHOOTER_RPM) > SHOOTER_READY_TOLERANCE * SHOOTER_RPM) {
                    shooterAtSpeed = false;
                }
            }

            // ------------------------------------------------ ball detection: the biggest (= closest) round blob
            Target target = findTarget(ballLocator);
            if (target == null) {
                seenTimer.reset();   // the ball must be in view without a gap to count as "seen"
            }

            boolean driverTurning = Math.abs(gamepad1.right_stick_x) > 0.1;
            boolean driverUsingSticks = driverTurning
                    || Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) > 0.1;

            // Aim at the middle of the chosen group; the distance is the one of the nearest ball in it.
            double aimErrorPx = target == null ? 0 : target.aimX - CAMERA_WIDTH / 2.0;
            double distanceCm = target == null ? 0 : target.closest.distanceCm();
            boolean tooCloseToMeasure = target != null && target.closest.touchesTopOrBottom();
            // 1 when the ball is centered, 0 at the picture's edge. Used to slow down until the robot faces the ball.
            double centeredFactor = Math.max(0, 1 - Math.abs(aimErrorPx) / (CAMERA_WIDTH / 2.0));

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double driveHeading = heading;   // field-centric while the driver is in control
            boolean autoTurning = false;     // the ball grab decides the turn power this loop

            // ------------------------------------------------ ball grab (HOME)
            if (driverUsingSticks && hitState != HitState.IDLE) {
                hitState = HitState.IDLE;   // touching the sticks cancels the automatic driving
                hitNote = "cancelled by driver";
            }
            if (homePressed) {
                if (hitState != HitState.IDLE) {
                    hitState = HitState.IDLE;
                    hitNote = "stopped";
                } else if (!driverUsingSticks) {
                    hitState = HitState.SEARCH;   // goes straight to the ball if one is already in view
                    hitNote = "";
                }
            }

            if (hitState != HitState.IDLE) {
                // Everything below drives in the robot's own frame (where the camera looks) and starts from "stand still".
                driveHeading = 0;
                x = 0;
                y = 0;
                turn = 0;

                if (hitState == HitState.LOST && target != null) {
                    hitState = HitState.APPROACH;   // found it again
                    ballLostTimer.reset();
                }
                // While searching, a ball must stay in view for LOCK_CONFIRM_SECONDS (robot standing still)
                // before the robot locks onto it. One flickering blob must not send the robot off.
                boolean confirming = false;
                if (hitState == HitState.SEARCH && target != null) {
                    if (seenTimer.seconds() >= LOCK_CONFIRM_SECONDS) {
                        hitState = HitState.APPROACH;
                        ballLostTimer.reset();
                    } else {
                        confirming = true;
                    }
                }

                if (hitState == HitState.APPROACH) {
                    boolean startPush = false;
                    if (target != null) {
                        ballLostTimer.reset();
                        lastBallDistanceCm = distanceCm;
                        lastBallErrorPx = aimErrorPx;
                        lastSeenHeadingDeg = headingDeg;
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
                        hitState = HitState.LOST; // it was far: go looking for it
                    }
                    if (startPush) {
                        hitState = HitState.PUSH;
                        pushHeadingDeg = headingDeg;
                        pushTimer.reset();
                        pushPid.reset();
                    }
                }

                if (hitState == HitState.PUSH) {
                    if (pushTimer.seconds() >= HIT_PUSH_SECONDS) {
                        hitsDone++;
                        hitNote = "ball hit";
                        if (SEARCH_AGAIN_AFTER_HIT) {
                            hitState = HitState.SEARCH;   // look for the next ball
                        } else {
                            hitState = HitState.IDLE;
                        }
                    } else {
                        // Drive straight, holding the heading the push started with.
                        y = HIT_PUSH_POWER;
                        double pushErrorDeg = AngleUnit.normalizeDegrees(pushHeadingDeg - headingDeg);
                        turn = Range.clip(-pushPid.update(pushErrorDeg), -MAX_PUSH_TURN_POWER, MAX_PUSH_TURN_POWER);
                    }
                }

                if (hitState == HitState.LOST) {
                    double turnedDeg = Math.abs(AngleUnit.normalizeDegrees(headingDeg - lastSeenHeadingDeg));
                    if (ballLostTimer.seconds() > LOST_SECONDS || turnedDeg > LOST_MAX_TURN_DEG) {
                        hitNote = "lost the ball";
                        if (SEARCH_AGAIN_AFTER_HIT) {
                            hitState = HitState.SEARCH;
                        } else {
                            hitState = HitState.IDLE;
                        }
                    } else {
                        // Ball left on the right (+ error) -> clockwise (+ turn). Near the center -> just wait,
                        // it probably flickered out or rolled under the camera.
                        turn = Math.abs(lastBallErrorPx) < LOST_EDGE_PX
                                ? 0 : Math.signum(lastBallErrorPx) * LOST_TURN_POWER;
                    }
                }

                if (hitState == HitState.SEARCH) {
                    // Spin around and around (full circles) until a ball is seen.
                    // Something is in view but not confirmed yet: stand still and look at it.
                    turn = confirming ? 0 : SEARCH_DIRECTION * SEARCH_TURN_POWER;
                }

                // IDLE (the ball grab just ended) stands still, so heading hold may keep the robot pointing the same way.
                autoTurning = hitState != HitState.IDLE;
            }

            // ------------------------------------------------ heading hold
            // If nobody is turning the robot on purpose, keep it pointing the same way.
            if (driverTurning || autoTurning || !HEADING_HOLD_ENABLED) {
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

            // ------------------------------------------------ mecanum drive (sin/cos)
            double stickAngle = Math.atan2(y, x);
            // Field-centric = subtract the heading.
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

            // ------------------------------------------------ telemetry (short on purpose)
            String intakeText;
            if (intakeState == IntakeState.COLLECTING) {
                intakeText = "COLLECTING (DPAD DOWN = stop, DPAD UP = shoot)";
            } else if (intakeState == IntakeState.SHOOTING) {
                intakeText = "SHOOTING (DPAD UP = end)";
            } else if (grabIntake) {
                intakeText = "COLLECTING (on by itself during the ball grab)";
            } else {
                intakeText = "off (DPAD DOWN = collect, DPAD UP = shoot)";
            }
            telemetry.addData("intake", intakeText);

            StringBuilder shooterText = new StringBuilder();
            for (int i = 0; i < shooters.length; i++) {
                shooterText.append(String.format("%s %.0f rpm%s   ", shooterNames[i], shooterRpm[i],
                        shooterNoEncoder[i] ? " (NO ENCODER! plain power)" : ""));
            }
            if (SHOOTER_FULL_POWER) {
                shooterText.append("FULL POWER");
            } else {
                shooterText.append(String.format("target %.0f  %s", SHOOTER_RPM, shooterAtSpeed ? "AT SPEED" : "spinning up / recovering"));
            }
            telemetry.addData("shooter", shooterText.toString());

            String ballText;
            switch (hitState) {
                case SEARCH:   ballText = (target != null ? "checking what it sees..." : "searching...")
                                       + " (HOME = stop)"; break;
                case APPROACH: ballText = target == null ? "driving at the ball (lost from view)"
                                       : String.format("driving at the ball ~%.0f cm", distanceCm); break;
                case LOST:     ballText = "ball lost, turning to find it again"; break;
                case PUSH:     ballText = "HITTING the ball"; break;
                default:       ballText = (hitNote.isEmpty() ? "ready" : hitNote) + " (HOME = go for the balls)"; break;
            }
            telemetry.addData("ball grab", ballText + "   hits so far: " + hitsDone);
            // What the camera sees. If the robot drives at nothing, look here: a huge "biggest" or many blobs
            // means the yellow range is catching more than the balls.
            telemetry.addData("camera sees", String.format("%d yellow blob(s), biggest %.0f px, roundness %.2f -> %s",
                    rawBlobCount, rawBiggestArea, rawBiggestCircularity,
                    target == null ? "no ball"
                            : String.format("%d ball(s) in the best group, nearest ~%.0f cm", target.count, distanceCm)));
            telemetry.addData("white balance", String.format("red x%.2f  green x%.2f  blue x%.2f  (measured on %.0f%% of the picture)",
                    colorFix.getRedGain(), colorFix.getGreenGain(), colorFix.getBlueGain(),
                    colorFix.getMeasuredShare() * 100));
            addColorProbeTelemetry(colorProbe);
            telemetry.addData("heading", holding
                    ? String.format("%.0f deg, holding %.0f", headingDeg, holdHeadingDeg)
                    : String.format("%.0f deg", headingDeg));
            telemetry.update();
        }

        visionPortal.close();
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

    // The color in the white box in the middle of the stream, and whether it is inside BALL_COLOR.
    void addColorProbeTelemetry(ColorSampler probe) {
        boolean inside = probe.getY() >= BALL_MIN_Y && probe.getY() <= BALL_MAX_Y
                && probe.getCr() >= BALL_MIN_CR && probe.getCr() <= BALL_MAX_CR
                && probe.getCb() >= BALL_MIN_CB && probe.getCb() <= BALL_MAX_CB;
        telemetry.addData("color in the white box", String.format("Y %.0f  Cr %.0f  Cb %.0f  -> %s",
                probe.getY(), probe.getCr(), probe.getCb(), inside ? "BALL COLOR" : "not ball color"));
    }

    // Picks what to drive at: among all round blobs of the right color and a sensible size, the biggest GROUP of
    // balls (balls whose centers are within CLUSTER_DISTANCE_PX of each other; most ball area wins).
    // Returns null if there is no ball. Also fills the rawBlob* numbers for the telemetry.
    Target findTarget(ColorBlobLocatorProcessor locator) {
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(locator.getBlobs());
        rawBlobCount = blobs.size();
        rawBiggestArea = 0;
        rawBiggestCircularity = 0;
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            if (blob.getContourArea() > rawBiggestArea) {
                rawBiggestArea = blob.getContourArea();
                rawBiggestCircularity = blob.getCircularity();
            }
        }

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, BALL_MIN_AREA, BALL_MAX_AREA, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, BALL_MIN_CIRCULARITY, 1, blobs);
        if (blobs.isEmpty()) {
            return null;
        }

        // For each ball: how much ball area sits around it (itself included)? The best ball marks the best group.
        ColorBlobLocatorProcessor.Blob bestCenter = null;
        double bestScore = -1;
        for (ColorBlobLocatorProcessor.Blob a : blobs) {
            double score = 0;
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                if (distancePx(a, b) <= CLUSTER_DISTANCE_PX) {
                    score += b.getContourArea();
                }
            }
            if (score > bestScore) {
                bestScore = score;
                bestCenter = a;
            }
        }

        // The group = every ball near that one. Aim at the group's middle (weighted by size), measure the nearest ball.
        Ball closest = null;
        double weightedX = 0;
        double totalArea = 0;
        int count = 0;
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            if (distancePx(bestCenter, b) <= CLUSTER_DISTANCE_PX) {
                count++;
                weightedX += b.getCircle().getX() * b.getContourArea();
                totalArea += b.getContourArea();
                if (closest == null || b.getContourArea() > closest.blob.getContourArea()) {
                    closest = new Ball(b);
                }
            }
        }
        return new Target(closest, weightedX / totalArea, count);
    }

    static double distancePx(ColorBlobLocatorProcessor.Blob a, ColorBlobLocatorProcessor.Blob b) {
        return Math.hypot(a.getCircle().getX() - b.getCircle().getX(), a.getCircle().getY() - b.getCircle().getY());
    }

    // What the robot drives at: the nearest ball of the best group, where to aim (x in the picture), and how many balls are in the group.
    static class Target {
        final Ball closest;
        final double aimX;
        final int count;

        Target(Ball closest, double aimX, int count) {
            this.closest = closest;
            this.aimX = aimX;
            this.count = count;
        }
    }

    // A detected ball: the blob the color locator found.
    static class Ball {
        final ColorBlobLocatorProcessor.Blob blob;

        Ball(ColorBlobLocatorProcessor.Blob blob) {
            this.blob = blob;
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
}
