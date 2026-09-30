package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Circle Button Test", group = "Test")
@Disabled
public class CircleButtonTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.circle) {
                telemetry.addLine("Circle is pressed");
            }
            telemetry.update();
        }
    }
}
