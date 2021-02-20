package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;

@TeleOp (name="testing and hardware calibration", group = "UltimateGoal")
public class testingAndHardwareCalibration extends LinearOpMode {
    /*declare Opmode members, initialize some classes*/
    HardwareUltimateGoal robot          = new HardwareUltimateGoal("testing");
    PositionAndTargetManager posTarMan  = new PositionAndTargetManager(robot, true); //TODO: 10/21/2020 change true to false if on team blue
    ElapsedTime runtime                 = new ElapsedTime();
    AimAssist aimMan                    = new AimAssist(robot, posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.bestTargetPosition(0));

    @Override
    public void runOpMode() {
        waitForStart();
        robot.init(hardwareMap);

        robot.intakePulley.setPower(1);

        sleep(1000);

        robot.intakePulley.setPower(-1);

        sleep(500);

        robot.intakePulley.setPower(0);
    }



    // utility classes
    /**
     * drives the robot forward a given distance, to go in reverse, give a negative value for power
     * @param left the motor on the left
     * @param right the motor on the right
     * @param distance the distance the robot should move, meters, always positive, IMPORTANT, You're passing the desired change, not a desired place (this makes more sense with the turn method)
     * @param power the power the robot should move at
     */
    public void encoderDrive(DcMotor left, DcMotor right, double distance, double power) {
        distance *= robot.NADO_COUNTS_PER_METER; //converts the desired distance into encoder ticks
        //if they want to move backwards, invert distance
        if (power < 0) {
            distance *= -1;
        }

        //set up right and left to run to position
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setTargetPosition((int) (left.getCurrentPosition() + distance));
        left.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setTargetPosition((int) (right.getCurrentPosition() + distance));
        right.setPower(0);

        //set to RunToPosition
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        // while the motors are busy, update the positionTargetManager
        int countOffset = left.getCurrentPosition() - right.getCurrentPosition();
        boolean leftBigger = true;
        if (countOffset < 0) {
            leftBigger = false;
            countOffset *= -1;
        }
        while (right.isBusy() || left.isBusy()) {
            int leftPos = left.getCurrentPosition();
            int rightPos = right.getCurrentPosition();
            posTarMan.update(left.getCurrentPosition(), right.getCurrentPosition());

            // pid type stuff i think
            if (leftBigger) leftPos -= countOffset;
            else            rightPos -=countOffset;

            if (leftPos > rightPos) {
                left.setPower(power - 0.05);
                right.setPower(power + 0.05);
            } else if (leftPos < rightPos) {
                left.setPower(power + 0.05);
                right.setPower(power - 0.05);
            } else {
                left.setPower(power);
                right.setPower(power);
            }
            telemetry.addData("left encoder count", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right encoder count", robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }

        //stop running to position
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

