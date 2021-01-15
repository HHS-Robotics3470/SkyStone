package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * basic autonomous, will probably be dead reckoning.
 *
 * after every step of the autonomous, make a call to update the position manager
 */
@Autonomous(name="Basic Autonomous Ultimate Goal", group="UltimateGoal")
public class BasicAutonomousUltimateGoal extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareUltimateGoal robot          = new HardwareUltimateGoal("testing");
    PositionAndTargetManager posTarMan  = new PositionAndTargetManager(robot, HardwareUltimateGoal.readPosition(), HardwareUltimateGoal.readHeading(), true); //TODO: 10/21/2020 change true to false if on team blue
    ElapsedTime runtime                 = new ElapsedTime();
    AimAssist aimMan                    = new AimAssist(robot, posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.bestTargetPosition(0));

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //set up other things
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Step through each leg of the path,

        //---------------------------------------------------------------------------------------\\

        // PARAMETER VALUES ARE TENTATIVE AS EXPERIMENTATION IS NEEDED TO FINE TUNE AND ADJUST THESE VALUES \\

        //move forward to right before the launch line
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        sleep(2000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //turn left
        robot.leftDrive.setPower(-0.5);
        robot.rightDrive.setPower(0.5);
        sleep(1000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //move forward, where the starting stack is now to the left of the robot
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        sleep(500);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //turn right, where the starting stack is now behind the robot
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(-0.5);
        sleep(1000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //shoot ring 1 into the high goal
        robot.flyWheel1.setPower(1);
        robot.flyWheel2.setPower(1);
        sleep(500);
        robot.flyWheel1.setPower(0);
        robot.flyWheel2.setPower(0);

        //reloads ring 2 into the chamber
        robot.conveyor.setPower(1);
        sleep(2000);
        robot.conveyor.setPower(0);

        //shoot ring 2 into the high goal
        robot.flyWheel1.setPower(1);
        robot.flyWheel2.setPower(1);
        sleep(500);
        robot.flyWheel1.setPower(0);
        robot.flyWheel2.setPower(0);

        for(int step = 0; step < 3; step++) {

            //back up and take in a ring from the starting stack
            robot.intakePulley.setPower(1);
            robot.leftDrive.setPower(-0.25);
            robot.rightDrive.setPower(-0.25);
            sleep(1000);
            robot.intakePulley.setPower(1);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            //move forward, to right behind the launch line
            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(0.5);
            sleep(500);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            //reloads a ring from the starting stack into the chamber
            robot.conveyor.setPower(1);
            sleep(2000);
            robot.conveyor.setPower(0);

            //shoot ring into the high goal
            robot.flyWheel1.setPower(1);
            robot.flyWheel2.setPower(1);
            sleep(500);
            robot.flyWheel1.setPower(0);
            robot.flyWheel2.setPower(0);
        }

        //drive over the launch line
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        sleep(500);
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);

        //Saves the robot's position and heading
        HardwareUltimateGoal.writePositionHeading(posTarMan.getRobotPosition(), posTarMan.getRobotHeading());

        //---------------------------------------------------------------------------------------\\
        /* new idea
        (while doing these steps, make sure to update the robots position frequently)

        step 1: move forward to right before the launch line, in front of the starting stack, without knocking them over

        step 2: fire the 2 rings we preloaded (either the high goal, or the power shots, we need to test the robots accuracy to figure out which is optimal

        step 3: move to the starting stack, and reload 1 ring from the starting stack

        step 4: move back to (right before) the launch line (optional), and fire the ring we just picked up

        step 5: (if we figure out that we'll have time, and be able to (if the stack gets scattered too much in step 4 this won't be possible), repeat steps 3-4 until we have fired 3 rings from the starting stack

        step 6: move ON TOP OFF the launch line, and end autonomous, (in the end step, make sure to accurately store the robots position and heading using the writePositionHeading(double[] position, double heading) method of the HardwareUltimateGoal.java class
         */

        /* old:
        step 1: pick up and store the rings in the starter stack
            (if we get a way to know how many rings where in the starter stack, do this before doing step 2)
            step 1.5: wobble goal stuff
        step 2: turn parallel to goal tower, and move until aligned with the goal tower, then turn to face the goal tower

            if we have a color sensor, do step 3 b, otherwise do step 3 a
        step 3.a: use the position manager to find the launch line, then move forward until the robot as close as it can get to the launch line without moving over it
        step 3.b: move forward until color sensor finds launch line, then stop, and reverse so the robot is no longer over launch line

        step 4: using the aim manager, cycle through the targets (top to bottom) until it finds a target it can hit
            (if it can't hit any, move forward over the launch line, and fire into the low goal
        step 5: go through the launch cycle, with a small pause between launches to give the flywheels time to get to speed
            repeat step 5 once per ring in the starter stack, if we don't get a way to count that, then just fire 3 times

        step 6: move the robot over the launch line (on top of launch line), and save the robots heading and position
         */
    }
    //TODO: eventually make a program that lets the robot move in arcs or something
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
        }

        //stop running to position
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     *
     * @param left the motor on the left
     * @param right the motor on the right
     * @param angle the desired change in angle, in radians, positive angles turn ccw, negative angles turn cw
     * @param power the power the motors should move at, should always be positive
     */
    public void encoderTurn(DcMotor left, DcMotor right, double angle, double power) {
        double wheelCircumference = robot.NADO_WHEEL_DIAMETER_METERS * Math.PI;
        double turnCircumference = robot.robotWidth * Math.PI;

        double angleCircumference = angle * robot.robotWidth;//in meters
        angleCircumference *= robot.NADO_COUNTS_PER_METER; // in encoder ticks

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
            posTarMan.update(left.getCurrentPosition(), right.getCurrentPosition());
        }

        //stop running to position
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
