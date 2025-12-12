package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class ReadObelisk {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode opMode;

    public ReadObelisk (RobotHardware robot, Telemetry telemetry, LinearOpMode opMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }
    public patterns getPattern() {
        patterns patternFound = patterns.PPG;  // default
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        LLResult result = robot.limelight.getLatestResult();
        while(result.isValid() == false && opMode.opModeIsActive() && runtime.milliseconds() < 500) {
            result = robot.limelight.getLatestResult();
            telemetry.addData("result is Valid", result.isValid());
            telemetry.update();
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            telemetry.update();
            if (fr.getFiducialId() == 21) patternFound = patterns.GPP;
            if (fr.getFiducialId() == 22) patternFound = patterns.PGP;
            if (fr.getFiducialId() == 23) patternFound = patterns.PPG;
        }
        return patternFound;
    }

    public double getOffset() {
        double offset = 0;  // default
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        LLResult result = robot.limelight.getLatestResult();
        while(result.isValid() == false && runtime.milliseconds() < 500) {
            result = robot.limelight.getLatestResult();
            telemetry.addData("result is Valid", result.isValid());
            telemetry.update();
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            offset = fr.getTargetXDegrees();
            telemetry.update();
        }
        return offset;
    }
}
