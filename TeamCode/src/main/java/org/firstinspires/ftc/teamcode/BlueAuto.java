package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: BlueAuto", group="Robot")
public class BlueAuto extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry);
    Shoot shoot = new Shoot(robot, telemetry, this);
    ReadObelisk readObelisk = new ReadObelisk(robot, telemetry, this);
    Drive drive = new Drive(robot, telemetry, this);
    Load load = new Load(robot, telemetry, this);

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
        pattern = readObelisk.getPattern();
        drive.pid_forward(138, .6);
        RobotHardware.LIMELIGHT_PIPE = 2;
        robot.setLaunchSpeed(.75);  // spin up launch while turning
        robot.setAngle(.14);
        gyroTurn.goodEnough(41); // towards goal
        shoot.thePattern(pattern);
        robot.setLaunchSpeed(0);
        gyroTurn.goodEnough(-86);

        robot.setIntakeSpeed(1);
        robot.setRevolverPosition(robot.LOAD_3);
        drive.backward(34,.3);
        load.threeBalls(robot.LOAD_3, robot.LOAD_1, robot.LOAD_2);
        robot.setLaunchSpeed(.75);
        drive.forward(60,.55);
        robot.setIntakeSpeed(0);
        //sleep(30000);

        gyroTurn.goodEnough(36); // towards goal
        shoot.thePattern(pattern);
        gyroTurn.goodEnough(0);
        drive.backward(40,.5);

    }


}
