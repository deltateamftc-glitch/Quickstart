package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.PIDController;

// Test bench: a copy of the old Main TeleOp (drive + intake m0/m1 experiments). New ideas get tried here first;
// Final/MainTeleOp only gets what is proven. Not @Disabled on purpose, so it shows up on the Driver Station.
@TeleOp(name = "Test TeleOp", group = "Test")
public class TestTeleOp extends LinearOpMode {

    static final Rev9AxisImuOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP;
    static final Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection I2C_PORT_DIRECTION =
            Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD;

    static final double HEADING_KP = 0.005;
    static final double HEADING_KI = 0.02;
    static final double HEADING_KD = 0.002;
    static final double HEADING_I_ZONE_DEG = 5;
    static final double HEADING_TOLERANCE_DEG = 1;
    static final double HEADING_SETTLED_DEG_PER_SEC = 10;
    static final double MAX_AUTO_TURN_POWER = 0.6;

    // Heading hold: when the driver isn't turning, keep the robot pointing the same way
    // (uneven wheel drag would otherwise slowly rotate it while driving).
    static final boolean HEADING_HOLD_ENABLED = true;
    static final double HOLD_DEADBAND_DEG = 1;           // errors smaller than this get no angle correction
    static final double HOLD_KP = 0.02;                  // turn power per degree beyond the deadband (grows smoothly from 0)
    static final double HOLD_KD = 0.001;                 // turn power per deg/s of rotation, resists any drift right away
    static final double MAX_HOLD_TURN_POWER = 0.3;
    static final double HOLD_CAPTURE_DEG_PER_SEC = 20;   // after letting go of the turn stick, start holding once slower than this

    // SHARE switches ALL stabilization on/off: field-centric (headless) driving + heading hold.
    // Off = plain robot-centric driving, e.g. if the IMU acts up during a match.
    static final boolean STABILIZATION_ON_AT_START = true;

    // Intake motor "m0": tap DPAD DOWN = spin one way until tapped again, tap DPAD UP = spin the other way
    // until tapped again. Tapping the other button while it spins switches direction.
    static final double INTAKE_POWER = 0.7;

    // Second intake wheel, motor "m1": tap DPAD RIGHT = spin one way until tapped again, tap DPAD LEFT = spin the
    // other way until tapped again. Works the same as m0, but it is NOT part of the TRIANGLE (Y) move below.
    static final double INTAKE2_POWER = 0.7;

    // TRIANGLE (Y) = intake (m0 only) "unjam" move: a short first push, then a longer second push the opposite way.
    // First push = the DPAD DOWN direction, second push = the DPAD UP direction.
    static final double UNJAM_FIRST_SECONDS = 0.08;
    static final double UNJAM_SECOND_SECONDS = 0.5;

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

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "m0");   // "Ex" version can report current (amps)
        int intakeDirection = 0;         // 1 = DPAD DOWN way, -1 = DPAD UP way, 0 = stopped
        double intakePeakAmps = 0;       // highest current seen since INIT
        ElapsedTime unjamTimer = new ElapsedTime();   // time since TRIANGLE (Y) was pressed
        boolean unjamming = false;

        DcMotor intake2 = hardwareMap.get(DcMotor.class, "m1");   // second intake wheel
        int intake2Direction = 0;        // 1 = DPAD RIGHT way, -1 = DPAD LEFT way, 0 = stopped

        IMU imu = hardwareMap.get(IMU.class, "imu");
        boolean imuOk = imu.initialize(new IMU.Parameters(
                new Rev9AxisImuOrientationOnRobot(LOGO_DIRECTION, I2C_PORT_DIRECTION)));
        imu.resetYaw();

        PIDController headingPid = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_I_ZONE_DEG);
        boolean turningToZero = false;
        boolean holding = false;     // heading hold is active
        double holdHeadingDeg = 0;   // the heading heading hold keeps
        boolean stabilization = STABILIZATION_ON_AT_START;

        telemetry.addData("IMU initialized", imuOk);
        telemetry.addLine("Left stick = drive / strafe, right stick = turn");
        telemetry.addLine("CROSS (X) = turn back to 0 degrees");
        telemetry.addLine("OPTIONS = reset heading (point the robot away from you first)");
        telemetry.addLine("SHARE = stabilization (field-centric + heading hold) on/off");
        telemetry.addLine("DPAD DOWN / DPAD UP = intake (m0) one way / the other way, press the same one again to stop");
        telemetry.addLine(String.format("TRIANGLE (Y) = intake (m0) unjam: push %.2f s, then %.2f s the other way",
                UNJAM_FIRST_SECONDS, UNJAM_SECOND_SECONDS));
        telemetry.addLine("DPAD RIGHT / DPAD LEFT = second intake wheel (m1) one way / the other way, press the same one again to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.shareWasPressed()) {
                stabilization = !stabilization;
                holding = false;
                gamepad1.rumble(stabilization ? 300 : 1000);   // short buzz = on, long buzz = off
            }

            if (gamepad1.options) {
                imu.resetYaw();
                holding = false;   // the old held heading means nothing after a reset
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double headingDeg = Math.toDegrees(heading);
            double headingErrorDeg = AngleUnit.normalizeDegrees(0 - headingDeg);
            double turnRateDeg = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            if (gamepad1.crossWasPressed()) {
                turningToZero = true;
                headingPid.reset();
            }

            boolean driverTurning = Math.abs(gamepad1.right_stick_x) > 0.1;
            boolean settled = Math.abs(headingErrorDeg) < HEADING_TOLERANCE_DEG
                    && Math.abs(turnRateDeg) < HEADING_SETTLED_DEG_PER_SEC;
            if (driverTurning || settled) {
                turningToZero = false;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (driverTurning || turningToZero || !HEADING_HOLD_ENABLED || !stabilization) {
                holding = false;   // the driver or the auto-turn is in charge of turning
            } else if (!holding && Math.abs(turnRateDeg) < HOLD_CAPTURE_DEG_PER_SEC) {
                // The robot has (almost) stopped turning: remember this heading and keep it.
                holding = true;
                holdHeadingDeg = headingDeg;
            }
            double holdErrorDeg = holding ? AngleUnit.normalizeDegrees(holdHeadingDeg - headingDeg) : 0;

            if (turningToZero) {
                // Positive turn spins the robot clockwise, but heading grows counterclockwise, so flip the sign.
                turn = Range.clip(-headingPid.update(headingErrorDeg), -MAX_AUTO_TURN_POWER, MAX_AUTO_TURN_POWER);
            } else if (holding) {
                // Only the part of the error beyond the deadband counts, so the correction starts at 0 and grows smoothly.
                double beyondDeadbandDeg = Math.max(0, Math.abs(holdErrorDeg) - HOLD_DEADBAND_DEG);
                // Same sign flip as above. The rotation term works even inside the deadband:
                // turning counterclockwise (+ rate) is answered with clockwise (+) turn power.
                turn = -Math.signum(holdErrorDeg) * HOLD_KP * beyondDeadbandDeg + HOLD_KD * turnRateDeg;
                turn = Range.clip(turn, -MAX_HOLD_TURN_POWER, MAX_HOLD_TURN_POWER);
            }

            double stickAngle = Math.atan2(y, x);
            // Field-centric = subtract the heading. Stabilization off = robot-centric (sticks relative to the robot).
            double theta = stickAngle - (stabilization ? heading : 0);
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

            // Intake toggle. Call each WasPressed once per loop (it is consumed when read).
            boolean downTapped = gamepad1.dpadDownWasPressed();
            boolean upTapped = gamepad1.dpadUpWasPressed();
            boolean unjamTapped = gamepad1.triangleWasPressed();   // TRIANGLE = Y
            if (downTapped) {
                intakeDirection = intakeDirection == 1 ? 0 : 1;      // same way again = stop
                unjamming = false;                                   // a D-pad tap takes over from the unjam move
            } else if (upTapped) {
                intakeDirection = intakeDirection == -1 ? 0 : -1;
                unjamming = false;
            }
            if (unjamTapped) {
                unjamming = true;       // (pressing again while it runs starts it over)
                unjamTimer.reset();
            }

            double intakePower = intakeDirection * INTAKE_POWER;   // what the D-pad toggle wants
            if (unjamming) {
                double t = unjamTimer.seconds();
                if (t < UNJAM_FIRST_SECONDS) {
                    intakePower = INTAKE_POWER;
                } else if (t < UNJAM_FIRST_SECONDS + UNJAM_SECOND_SECONDS) {
                    intakePower = -INTAKE_POWER;
                } else {
                    unjamming = false;   // done: go back to what the D-pad toggle says (spinning or stopped)
                    intakePower = intakeDirection * INTAKE_POWER;
                }
            }
            intake.setPower(intakePower);

            // Second intake wheel (m1) toggle: same idea, on DPAD RIGHT / LEFT, no unjam move.
            boolean rightTapped = gamepad1.dpadRightWasPressed();
            boolean leftTapped = gamepad1.dpadLeftWasPressed();
            if (rightTapped) {
                intake2Direction = intake2Direction == 1 ? 0 : 1;     // same way again = stop
            } else if (leftTapped) {
                intake2Direction = intake2Direction == -1 ? 0 : -1;
            }
            double intake2Power = intake2Direction * INTAKE2_POWER;
            intake2.setPower(intake2Power);

            double intakeAmps = intake.getCurrent(CurrentUnit.AMPS);
            intakePeakAmps = Math.max(intakePeakAmps, intakeAmps);

            telemetry.addData("intake m0", unjamming ? String.format("UNJAM move, power %.2f", intakePower)
                    : intakeDirection == 0 ? "stopped"
                    : String.format("spinning (DPAD), power %.2f", intakePower));
            telemetry.addData("intake current (A)", "%.2f   peak %.2f", intakeAmps, intakePeakAmps);
            telemetry.addData("intake 2 m1", intake2Direction == 0 ? "stopped"
                    : String.format("spinning (DPAD), power %.2f", intake2Power));
            telemetry.addData("stabilization (SHARE)", stabilization
                    ? "ON  (field-centric + heading hold)" : "OFF (robot-centric, no heading hold)");
            telemetry.addData("auto turn to 0", turningToZero ? "ACTIVE" : "off");
            if (holding) {
                telemetry.addData("heading hold", "HOLDING %.1f deg, error %.1f deg", holdHeadingDeg, holdErrorDeg);
            } else {
                telemetry.addData("heading hold", !HEADING_HOLD_ENABLED || !stabilization ? "disabled"
                        : driverTurning || turningToZero ? "off while turning" : "waiting for the robot to stop turning");
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
}
