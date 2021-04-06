package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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




        encoderDrive(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, .5, 0.5);

        sleep(2000);

        encoderTurn(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, Math.toRadians(90), 0.5);

        sleep(2000);

        encoderDrive(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, 0.3, 0.5);

        sleep(2000);

        encoderTurn(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, Math.toRadians(90), 0.5);

        sleep(2000);

        while (opModeIsActive()) {
            telemetry.addData("distance detected", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        /*
        robot.intakePulley.setPower(1);

        sleep(1000);

        robot.intakePulley.setPower(-1);

        sleep(500);

        robot.intakePulley.setPower(0);
         */
    }



    // utility classes
    /**
     * drives the robot forward a given distance, to go in reverse, give a negative value for power
     * @param left the motor on the left
     * @param right the motor on the right
     * @param horiz the horizontal odometry encoder
     * @param distance the distance the robot should move, meters, always positive, IMPORTANT, You're passing the desired change, not a desired place (this makes more sense with the turn method)
     * @param power the power the robot should move at (positive to go forward, negative to go backwards)
     */ //TODO: test this
    public void encoderDrive(DcMotor left, DcMotor right, DcMotor leftOdo, DcMotor rightOdo, DcMotor horizOdo, double distance, double power) {
        distance *= robot.ODOMETRY_COUNTS_PER_METER; //converts the desired distance into encoder ticks
        //if they want to move backwards, invert distance
        if (power < 0) {
            distance *= -1;
        }

        int initLeft = leftOdo.getCurrentPosition() * robot.getLeftDirection();
        int initRight = rightOdo.getCurrentPosition() * robot.getRightDirection();
        int initHoriz = horizOdo.getCurrentPosition() * robot.getHorizDirection();

        // while the change in measured distance (from odometry encoders) is less than the desired change, move
        left.setPower(0);
        right.setPower(0);

        int deltaLeft  = (leftOdo.getCurrentPosition()  * robot.getLeftDirection())  - initLeft;
        int deltaRight = (rightOdo.getCurrentPosition() * robot.getRightDirection()) - initRight;
        int deltaHoriz = (horizOdo.getCurrentPosition() * robot.getHorizDirection()) - initHoriz;
        //while the absolute value of (average side change) - (desired change) is not less than or equal to (is greater than) allowed side count offset:
        while ( Math.abs( Math.abs((deltaLeft - deltaRight)/2) - (int)distance ) > robot.getSideOdoAllowedCountOffset() ) {
            posTarMan.update(leftOdo.getCurrentPosition(), rightOdo.getCurrentPosition(), horizOdo.getCurrentPosition());
            //update delta__'s
            deltaLeft  = (leftOdo.getCurrentPosition()  * robot.getLeftDirection())  - initLeft;
            deltaRight = (rightOdo.getCurrentPosition() * robot.getRightDirection()) - initRight;
            deltaHoriz = (horizOdo.getCurrentPosition() * robot.getHorizDirection()) - initHoriz;

            //odometry telemetry
            telemetry.addLine("motors encoder readings");
            telemetry.addData("left", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right", robot.rightDrive.getCurrentPosition());

            telemetry.addLine("odometry readings");
            telemetry.addData("left odo", robot.leftOdometry.getCurrentPosition());
            telemetry.addData("right odo", robot.rightOdometry.getCurrentPosition());
            telemetry.addData("horizontal odo", robot.horizOdometry.getCurrentPosition());

            telemetry.update();


            //move
            //make sure it's not drifting
            //if there is a noticeable difference in the distance travelled by each side OR the horiz encoder detects too much of a change in angle
            if ( Math.abs(deltaLeft-deltaRight) > robot.getSideOdoAllowedCountOffset() || deltaHoriz > robot.getHorizOdoAllowedCountOffset()) {
                if (deltaLeft>deltaRight) {left.setPower(power-0.5);right.setPower(power);}         //if left has gone further, make it go slower
                else if (deltaLeft<deltaRight) {left.setPower(power-0.5);right.setPower(power);}    //if right has gone further, make it go slower
            } else {left.setPower(power);right.setPower(power);}
        }

        /*
        //vvv old vvv
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
            posTarMan.update(leftOdo.getCurrentPosition(), rightOdo.getCurrentPosition(), horizOdo.getCurrentPosition());

            /*//* pid type stuff i think
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
            }*//*
            telemetry.addData("left encoder count", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right encoder count", robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }
        //stop running to position
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
    }

    /**
     *
     * @param left the motor on the left
     * @param right the motor on the right
     * @param leftOdo the left odometry encoder
     * @param rightOdo the right odometry ecododer
     * @param horizOdo the horizontal odometry encoder
     * @param angle the desired change in angle, in radians, should always be positive
     * @param power the power the motors should move at, positive turn ccw, angles turn cw
     */ //TODO: test this
    public void encoderTurn(DcMotor left, DcMotor right, DcMotor leftOdo, DcMotor rightOdo, DcMotor horizOdo, double angle, double power) {
        /* old from pre-odo times
        double wheelCircumference = robot.ODOMETRY_WHEEL_DIAMETER_METERS * Math.PI;
        double turnCircumference = robot.getRobotOdometryWidth() * Math.PI;
        double angleCircumference = angle * robot.getRobotOdometryLength();//in meters //arc length formula
        angleCircumference *= robot.ODOMETRY_COUNTS_PER_METER; // in encoder ticks
        */
        // target heading
        double targetHeading = posTarMan.getRobotHeading();
        if (power > 0) targetHeading += angle;
        else targetHeading -= angle;
        //orientate the target heading
        double twoPI = 2 * Math.PI;
        if (targetHeading > Math.PI) {
            targetHeading = -twoPI + targetHeading;
        } else if (targetHeading < -Math.PI) {
            targetHeading = twoPI + targetHeading;
        }

        left.setPower(-power);
        right.setPower(power);
        //while the current heading is too far from the target heading, move
        while ( Math.abs(targetHeading - posTarMan.getRobotHeading()) > robot.getHorizOdoAllowedCountOffset()) {
            posTarMan.update(leftOdo.getCurrentPosition(), rightOdo.getCurrentPosition(), horizOdo.getCurrentPosition());


            //odometry telemetry
            telemetry.addLine("motors encoder readings");
            telemetry.addData("left", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right", robot.rightDrive.getCurrentPosition());

            telemetry.addLine("odometry readings");
            telemetry.addData("left odo", robot.leftOdometry.getCurrentPosition());
            telemetry.addData("right odo", robot.rightOdometry.getCurrentPosition());
            telemetry.addData("horizontal odo", robot.horizOdometry.getCurrentPosition());

            telemetry.update();

        }
        //once we're at the target position, exit loop and stop
        left.setPower(0);
        right.setPower(0);

        /*old, from pre odo
        //set up right and left to run to position
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setTargetPosition((int) (left.getCurrentPosition() - angleCircumference));
        left.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setTargetPosition((int) (right.getCurrentPosition() + angleCircumference));
        right.setPower(0);
        //set to RunToPosition
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //make sure the set powers will make the robot turn properly
        if (angle > 0) {
            left.setPower(-power);
            right.setPower(power);
        } else {
            left.setPower(power);
            right.setPower(-power);
        }
        // while the motors are busy, update the positionTargetManager
        while (right.isBusy() || left.isBusy()) {
            posTarMan.update(left.getCurrentPosition(), right.getCurrentPosition(), horiz.getCurrentPosition());
            telemetry.addData("left encoder count", robot.leftDrive.getCurrentPosition());
            telemetry.addData("right encoder count", robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }
        //stop running to position
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */
    }
}

