package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor ID Test", group = "Test")
@Disabled
public class MotorIdTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor motorCH0 = hardwareMap.get(DcMotor.class, "mec0");
        DcMotor motorCH2 = hardwareMap.get(DcMotor.class, "mec2");
        DcMotor motorEH1 = hardwareMap.get(DcMotor.class, "mec1");
        DcMotor motorEH3 = hardwareMap.get(DcMotor.class, "mec3");

        telemetry.addLine("Put the robot on a stand - the wheels will spin!");
        telemetry.addLine();
        telemetry.addLine("RIGHT BUMPER = all four, different speeds");
        telemetry.addLine("CROSS = only CH0      SQUARE = only CH2");
        telemetry.addLine("TRIANGLE = only EH1   CIRCLE = only EH3");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double ch0 = 0, ch2 = 0, eh1 = 0, eh3 = 0;

            if (gamepad1.right_bumper) {
                ch0 = 0.25;
                ch2 = 0.50;
                eh1 = 0.75;
                eh3 = 1.00;
            }
            if (gamepad1.cross)    ch0 = 0.50;
            if (gamepad1.square)   ch2 = 0.50;
            if (gamepad1.triangle) eh1 = 0.50;
            if (gamepad1.circle)   eh3 = 0.50;

            motorCH0.setPower(ch0);
            motorCH2.setPower(ch2);
            motorEH1.setPower(eh1);
            motorEH3.setPower(eh3);

            telemetry.addLine("--- speed test assignments ---");
            telemetry.addData("Control Hub port 0", "25%  (slowest)");
            telemetry.addData("Control Hub port 2", "50%");
            telemetry.addData("Expansion Hub port 1", "75%");
            telemetry.addData("Expansion Hub port 3", "100% (fastest)");
            telemetry.addLine();
            telemetry.addLine("--- live power ---");
            telemetry.addData("CH0", ch0);
            telemetry.addData("CH2", ch2);
            telemetry.addData("EH1", eh1);
            telemetry.addData("EH3", eh3);
            telemetry.update();
        }
    }
}
