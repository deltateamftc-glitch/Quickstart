package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum TeleOp", group = "Drive")
@Disabled
public class MecanumTeleOp extends LinearOpMode {

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

        telemetry.addLine("Left stick = drive / strafe, right stick = turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive  = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn   = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);

            double fl = (drive + strafe + turn) / denominator;
            double fr = (drive - strafe - turn) / denominator;
            double rl = (drive - strafe + turn) / denominator;
            double rr = (drive + strafe - turn) / denominator;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            rearLeft.setPower(rl);
            rearRight.setPower(rr);

            telemetry.addData("front left  (mec0)", "%.2f", fl);
            telemetry.addData("front right (mec1)", "%.2f", fr);
            telemetry.addData("rear left   (mec2)", "%.2f", rl);
            telemetry.addData("rear right  (mec3)", "%.2f", rr);
            telemetry.update();
        }
    }
}
