package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Robot: RedAutoGoal", group="Robot")
public class RedAutoGoal extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry);
    Shoot shoot = new Shoot(robot, telemetry, this);
    ReadObelisk readObelisk = new ReadObelisk(robot, telemetry, this);
    Drive drive = new Drive(robot, telemetry, this);
    Load load = new Load(robot, telemetry, this);

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    patterns pattern = patterns.PPG;


    @Override
    public void runOpMode() throws InterruptedException {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        robot.init_auto();

        robot.resetDriveEncoders();
        robot.limelight.start();

        // Wait for the game to start (driver presses START)

        waitForStart();
        robot.setLaunchSpeed(.78);  // spin up launch while turning
        robot.setAngle(.15);
        drive.backward(85, .5);
        gyroTurn.goodEnough(45);
        pattern = readObelisk.getPattern();
        gyroTurn.goodEnough(0);
        robot.LIMELIGHT_PIPE = 1;
        shoot.thePattern(pattern);
        gyroTurn.goodEnough(50);
        robot.setIntakeSpeed(0);
        robot.setLaunchSpeed(0);
        robot.setRevolverPosition(robot.LOAD_1);
        drive.backward(61, .25);


    }
}
