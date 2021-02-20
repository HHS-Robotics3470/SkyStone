package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class driveEncoderCalibration extends LinearOpMode {
    HardwareUltimateGoal robot = new HardwareUltimateGoal("calibrating");

    @Override
    public void runOpMode() throws InterruptedException {
        //move the robot a bit
        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1);

        sleep(1000);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //calculate the offset, the right drive is the motor we'll offset, the left drive is the one that slips,
            // find the difference between left and right
        int rightCount = robot.rightDrive.getCurrentPosition();
        int leftCount = robot.leftDrive.getCurrentPosition();

        // l = r * x
        double rightOffsetMultiplier = leftCount / rightCount;

        sleep(1000);

        //test if the new offset works
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftDrive.setPower(1);
        robot.rightDrive.setPower(1 * rightOffsetMultiplier);

        sleep(1000);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        int difference = Math.abs(robot.leftDrive.getCurrentPosition() - robot.rightDrive.getCurrentPosition());
        if ( difference < 50) { //if the difference is less than
            telemetry.addLine("success");
            telemetry.addData("difference", difference);
            telemetry.update();

            sleep(2500);

            //write the offset to a file
            HardwareUltimateGoal.writeRightDriveEncoderOffsetMultiplier(rightOffsetMultiplier);
        } else {
            telemetry.addLine("failed");
            telemetry.update();

            sleep(2500);

            HardwareUltimateGoal.writeRightDriveEncoderOffsetMultiplier(1.0);
        }
    }
}
