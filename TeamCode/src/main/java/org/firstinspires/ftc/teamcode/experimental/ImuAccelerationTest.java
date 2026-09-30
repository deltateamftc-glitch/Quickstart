package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "IMU Acceleration Test", group = "Test")
@Disabled
public class ImuAccelerationTest extends LinearOpMode {

    // The SDK puts the BNO055 in m/s^2 mode, where the chip reports 100 counts per m/s^2.
    static final double COUNTS_PER_MPS2 = 100.0;

    @Override
    public void runOpMode() {
        Rev9AxisImu imu = hardwareMap.get(Rev9AxisImu.class, "imu");
        boolean fusionRunning = imu.initialize(new IMU.Parameters(
                new Rev9AxisImuOrientationOnRobot(
                        Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP,
                        Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD)));
        I2cDeviceSynchSimple i2c = imu.getDeviceClient();

        telemetry.addData("IMU fusion running", fusionRunning);
        telemetry.addLine("Press START, keep the robot still, then push it around");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double[] linear  = readVector(i2c, BNO055IMU.Register.LIA_DATA_X_LSB);
            double[] overall = readVector(i2c, BNO055IMU.Register.ACC_DATA_X_LSB);
            double magnitude = Math.sqrt(overall[0] * overall[0]
                    + overall[1] * overall[1]
                    + overall[2] * overall[2]);

            telemetry.addData("IMU fusion running", fusionRunning);
            telemetry.addLine();
            telemetry.addLine("--- linear acceleration (gravity removed) m/s^2 ---");
            telemetry.addData("linear X", "%.2f", linear[0]);
            telemetry.addData("linear Y", "%.2f", linear[1]);
            telemetry.addData("linear Z", "%.2f", linear[2]);
            telemetry.addLine();
            telemetry.addLine("--- overall acceleration (with gravity) m/s^2 ---");
            telemetry.addData("overall X", "%.2f", overall[0]);
            telemetry.addData("overall Y", "%.2f", overall[1]);
            telemetry.addData("overall Z", "%.2f", overall[2]);
            telemetry.addData("magnitude (~9.8 when still)", "%.2f", magnitude);
            telemetry.update();
        }
    }

    private static double[] readVector(I2cDeviceSynchSimple i2c, BNO055IMU.Register firstRegister) {
        byte[] b = i2c.read(firstRegister.bVal, 6);
        return new double[] {
                (short) ((b[1] << 8) | (b[0] & 0xFF)) / COUNTS_PER_MPS2,
                (short) ((b[3] << 8) | (b[2] & 0xFF)) / COUNTS_PER_MPS2,
                (short) ((b[5] << 8) | (b[4] & 0xFF)) / COUNTS_PER_MPS2,
        };
    }
}
