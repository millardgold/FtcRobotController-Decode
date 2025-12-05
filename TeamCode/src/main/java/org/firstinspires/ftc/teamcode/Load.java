package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Load {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode  opMode;
    Drive drive;


    public Load (RobotHardware robot, Telemetry telemetry, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
        drive = new Drive(robot, telemetry, opMode);
    }
    public void threeBalls (double first, double second, double third) throws InterruptedException {
        robot.setIntakeSpeed(1);
        if (opMode.opModeIsActive()) {
            drive.backward(15,.1);
        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(second);
            Thread.sleep(1000);
            drive.backward(8,.1);
        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(third);
            Thread.sleep(1000);
            drive.backward(10,.1);
        }
    }

}