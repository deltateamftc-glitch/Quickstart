package org.firstinspires.ftc.teamcode.Final;

import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name = "Main TeleOp", group = "Final")
public class MainTeleOp extends LinearOpMode {

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

        telemetry.addData("IMU initialized", imuOk);
        telemetry.addLine("Left stick = drive / strafe, right stick = turn");
        telemetry.addLine("CROSS (X) = turn back to 0 degrees");
        telemetry.addLine("OPTIONS = reset heading (point the robot away from you first)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double headingErrorDeg = AngleUnit.normalizeDegrees(0 - Math.toDegrees(heading));
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

            if (turningToZero) {
                // Positive turn spins the robot clockwise, but heading grows counterclockwise, so flip the sign.
                turn = Range.clip(-headingPid.update(headingErrorDeg), -MAX_AUTO_TURN_POWER, MAX_AUTO_TURN_POWER);
            }

            double stickAngle = Math.atan2(y, x);
            double theta = stickAngle - heading;
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

            telemetry.addData("auto turn to 0", turningToZero ? "ACTIVE" : "off");
            telemetry.addData("robot heading (deg)", "%.1f", Math.toDegrees(heading));
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
