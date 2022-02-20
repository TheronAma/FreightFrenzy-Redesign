package org.firstinspires.ftc.teamcode.control;

public class PIDController {
    private double kP, kI, kD;
    private double error, totalError, lastError;

    private double target;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.target = 0;
        this.error = 0;
        this.totalError = 0;
        this.lastError = 0;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double update(double current) {
        error = target - current;
        totalError += error;
        totalError *= 0.8;
        double ret = error * kP + (error - lastError) * kD + totalError * kI;

        if(error > 0 && lastError < 0 || error < 0 && lastError > 0) {
            totalError = 0;
        }

        lastError = error;
        return ret;
    }
}
