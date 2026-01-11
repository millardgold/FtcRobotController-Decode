package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    double Kp;
    double Ki;
    double Kd;
    Telemetry telemetry;
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd, Telemetry telemetry) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.telemetry = telemetry;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param current where we currently are, I.E. motor position
     * @return the power to our motor, I.E. motor power
     */
    public double update(double target, double current) {
        double power;
        power = (current - target) * Kp;
//        telemetry.addData("target", target);
//        telemetry.addData("current", current);
//        telemetry.addData("kp", Kp);
//        telemetry.update();
        // PID logic and then return the output
        return power;
    }
}
