package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double integralZone;

    private final ElapsedTime timer = new ElapsedTime();
    private double integral;
    private double lastError;
    private boolean firstUpdate = true;

    public PIDController(double kP, double kI, double kD, double integralZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralZone = integralZone;
    }

    public void reset() {
        integral = 0;
        firstUpdate = true;
    }

    public double update(double error) {
        double dt = timer.seconds();
        timer.reset();

        if (firstUpdate) {
            firstUpdate = false;
            lastError = error;
            return kP * error;
        }

        // Only accumulate near the target, so a big starting error can't wind the integral up.
        if (Math.abs(error) < integralZone) {
            integral += error * dt;
        } else {
            integral = 0;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }
}
