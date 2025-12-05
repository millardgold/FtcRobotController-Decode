package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shoot {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode  opMode;

    public Shoot (RobotHardware robot, Telemetry telemetry, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public void thePattern (patterns pattern) throws InterruptedException {
        switch (pattern) {
            case PPG:
                telemetry.addData("PPG", pattern);
                telemetry.update();
                threeBalls(robot.LAUNCH_1, robot.LAUNCH_3, robot.LAUNCH_2);
                break;
            case PGP:
                telemetry.addData("PGP", pattern);
                telemetry.update();
                threeBalls(robot.LAUNCH_1, robot.LAUNCH_2, robot.LAUNCH_3);
                break;
            case GPP:
                telemetry.addData("GPP", pattern);
                telemetry.update();
                threeBalls(robot.LAUNCH_2, robot.LAUNCH_1, robot.LAUNCH_3);
                break;
        }
    }
    public void threeBalls (double first, double second, double third) throws InterruptedException {
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(first);
            Thread.sleep(1000);
            kickball();
        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(second);
            Thread.sleep(1000);
            kickball();
        }
        if (opMode.opModeIsActive()) {
            robot.setRevolverPosition(third);
            Thread.sleep(1000);
            kickball();
        }
    }

    private void kickball() throws InterruptedException {
        robot.setKickerPosition(RobotHardware.KICK_POSITION);
        Thread.sleep(500);
        robot.setKickerPosition(RobotHardware.KICK_RESET);
        Thread.sleep(500);
    }
}
