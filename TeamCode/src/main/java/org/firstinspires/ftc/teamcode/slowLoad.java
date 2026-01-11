package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;


@Autonomous(name="Robot: SlowLoad", group="Robot")
public class slowLoad extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.

    // motor declaration, we use the
    // Ex version as it has velocity measurements
    DcMotor leftDrive;
    DcMotor rightDrive;
    PIDController leftControl = new PIDController(0.05,0,0, telemetry);
    // object for pin point computer
    public GoBildaPinpointDriver odo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // the string is the hardware map name
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");


        // use braking to slow the motor down faster
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();

        waitForStart();
        // loop that runs while the program should run.
        // position in encoder ticks where we would like the motor to run
        int targetPosition = 23 * 138;
        double maxSpeed = .5;

        while (opModeIsActive() &&
               rightDrive.getCurrentPosition() < targetPosition){
            // update pid controller

            odo.update();
            double power = leftControl.update(0, odo.getHeading(AngleUnit.DEGREES));
            // assign motor the PID output
            if (power > 0) {
                leftDrive.setPower(maxSpeed + power);
                rightDrive.setPower(maxSpeed - power);
                telemetry.addData("leftPower", maxSpeed - power);
                telemetry.addData("rightPower", maxSpeed + power);
            }
            else {
                leftDrive.setPower(maxSpeed + power);
                rightDrive.setPower(maxSpeed - power);
                telemetry.addData("leftPower", maxSpeed - power);
                telemetry.addData("rightPower", maxSpeed + power);
            }
            telemetry.addData("Right Drive Pos", rightDrive.getCurrentPosition());
            telemetry.update();


//            double rightPower = control.update(500,
//                    rightDrive.getCurrentPosition());
//            // assign motor the PID output
//            rightDrive.setPower(rightPower);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(3000);
    }
}

