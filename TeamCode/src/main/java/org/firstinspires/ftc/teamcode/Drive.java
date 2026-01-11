package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class Drive {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode opMode;
    PIDController PID_controller = new PIDController(.05, 0, 0, telemetry);

    public Drive (RobotHardware robot, Telemetry telemetry, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }
    public void forward(double distance, double speed) {
        robot.resetDriveEncoders();
        robot.setTargets((int)(distance * robot.CLICKS_PER_CENTIMETER));
        robot.driveWhileBusy(speed, 3.0);
    }

    public void backward(double distance, double speed) {
        robot.resetDriveEncoders();
        robot.setTargets((int)-(distance * robot.CLICKS_PER_CENTIMETER));
        robot.driveWhileBusy(-speed, 3.0);
    }
    public void pid_forward(double distance, double maxSpeed) throws InterruptedException {
        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        robot.odo.update();
        double clicks = distance * RobotHardware.CLICKS_PER_CENTIMETER;
        robot.runWithoutEncoders();
        robot.leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        while (opMode.opModeIsActive() &&
                robot.leftDrive.getCurrentPosition() < clicks ){
            // update pid controller


            double power = PID_controller.update(0, robot.odo.getHeading(AngleUnit.DEGREES));
            // assign motor the PID output

            robot.setDrivePower(maxSpeed + power, maxSpeed - power);
            telemetry.addData("leftPower", maxSpeed - power);
            telemetry.addData("rightPower", maxSpeed + power);
            telemetry.update();
            robot.odo.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.resetDriveEncoders();
    }
}
