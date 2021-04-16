package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="odometry calibration", group="UltimateGoal")
public class driveEncoderCalibration extends LinearOpMode {
    HardwareUltimateGoal robot = new HardwareUltimateGoal("calibrating");

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        //move the robot a bit
        robot.leftDrive.setPower(-0.75);
        robot.rightDrive.setPower(-0.75);

        sleep(1000);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //calculate the offset, the left drive is the motor we'll offset, the right drive is the one that slips,
            // find the difference between left and right
        int rightCount = robot.rightOdometry.getCurrentPosition()*robot.getRightDirection();
        int leftCount = robot.leftOdometry.getCurrentPosition()*robot.getLeftDirection();

        // l = r * x
        double leftOffsetMultiplier = (double)rightCount / (double)leftCount;

        sleep(500);

        //test if the new offset works
        robot.leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftDrive.setPower(0.75 * leftOffsetMultiplier);
        robot.rightDrive.setPower(0.75);

        sleep(1000);

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        int difference = Math.abs(robot.leftOdometry.getCurrentPosition()*robot.getLeftDirection() - robot.rightOdometry.getCurrentPosition()*robot.getRightDirection());
        if ( difference < 200) { //if the difference is less than
            telemetry.addLine("success");
            telemetry.addData("difference", difference);
            telemetry.update();

            sleep(2500);

            //write the offset to a file
            HardwareUltimateGoal.writeLeftDriveEncoderOffsetMultiplier(leftOffsetMultiplier);
        } else {
            telemetry.addLine("failed");
            telemetry.addData("difference", difference);
            telemetry.update();

            sleep(2500);

            //HardwareUltimateGoal.writeRightDriveEncoderOffsetMultiplier(1.0);
        }
    }
}
