package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Square Button Test", group = "Test")
public class SquareButtonTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.square) {
                telemetry.addLine("Square is pressed");
            }
            telemetry.update();
        }
    }
}
