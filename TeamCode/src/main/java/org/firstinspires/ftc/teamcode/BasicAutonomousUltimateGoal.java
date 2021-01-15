package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.math.BigDecimal;
import java.math.MathContext;

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

        long timeToLowerIntake = 1000; //needs testing

        //set up other things
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Step through each leg of the path,

        //---------------------------------------------------------------------------------------\\

        // PARAMETER VALUES ARE TENTATIVE AS EXPERIMENTATION IS NEEDED TO FINE TUNE AND ADJUST THESE VALUES \\
        posTarMan.setTarget(3); //set the target to be the the high goal
        //move forward to right before the launch line
        encoderDrive(robot.leftDrive,robot.rightDrive,1.8, 1);
        /*robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        sleep(2000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);*/

        robot.intakePulley.setPower(-1);
        sleep(timeToLowerIntake);
        robot.intakePulley.setPower(0);

        //turn left
        encoderTurn(robot.leftDrive, robot.rightDrive, Math.toRadians(90), 1);
        /*robot.leftDrive.setPower(-0.5);
        robot.rightDrive.setPower(0.5);
        sleep(1000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);*/

        //move forward, where the starting stack is now to the left of the robot
        encoderDrive(robot.leftDrive,robot.rightDrive,0.28575, 1);
        /*robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        sleep(500);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);*/

        //turn right, where the starting stack is now behind the robot
        encoderTurn(robot.leftDrive,robot.rightDrive,-Math.toRadians(90),1);
        /*robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(-0.5);
        sleep(1000);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);*/

        //fire twice:
        aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());
        //  fire loaded ring
        fireTurret();
        //  reload
        reloadTurret();
        //  fire again
        fireTurret();

        //TODO: if we end up having enough time, put the next bit into a for loop that repeats at most 3 times
        //pick up another ring, then fire it
        encoderDrive(robot.leftDrive,robot.rightDrive,0.5382 - 0.0034,-1);
        robot.conveyor.setPower(1);
        encoderDrive(robot.leftDrive,robot.rightDrive,0.3,-0.2);
        sleep(4000);
        reloadTurret();
        encoderDrive(robot.leftDrive,robot.rightDrive,0.8382,1);
        fireTurret();
        /*//shoot ring 1 into the high goal
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

        //put down the intake pulley to enable the robot to take in rings from the starter stack
        robot.intakePulley.setPower(-1);
        sleep(3000);
        robot.intakePulley.setPower(0);

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
        robot.rightDrive.setPower(0.5);*/

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

    //////////////////////////////turret stuff//////////////////////////////
    //variables for the elevation iteration algorithm
    double elevationStep = 1;
    MathContext elevationMC = new MathContext(16);
    BigDecimal elevationLastGuessDegrees = new BigDecimal("00.0000", elevationMC);
    double elevationGuessOffset = 1;
    boolean firingError = false;
    boolean loaded = true;
    /**
     * given the angle relative to the field, convert to the angle relative to the robot (front = 0), then move the turret to that angle
     * @param angle angle relative to field
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and rotates the turret to the needed position
     */
    public int rotateTurretTo(double angle) {
        double targetPosition;
        double robotHeading = posTarMan.getRobotHeading();

        //heading relative to field -> heading relative to the robot
        targetPosition = angle - robotHeading;
        //check if heading rel. to robot is in the deadzone, if so, return -1
        if (targetPosition > Math.toRadians(45) || targetPosition < -Math.toRadians(45)) return -1;
        //convert the heading rel. to robot into the needed encoder counts
        targetPosition /= robot.CORE_HEX_RADIANS_PER_COUNTS;
        //make sure that the robot rotates the best direction to reach goal

        //rotate to that position and return 0
        robot.runMotorToPosition(robot.turretRotator, (int) targetPosition, 0.5);
        return 0;
    }
    /**
     * given the angle relative to the horizontal, move the turret to elevate to that pitch
     * @param angle desired angle of pitch
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and elevates the turret to the needed position
     */
    public double elevateTurretTo(double angle) {
        //iteration
        //reset the global values that control the iteration
        elevationStep = 1;
        elevationLastGuessDegrees = BigDecimal.ZERO;
        elevationGuessOffset = 1;
        //logic statement to make sure that the given target angle of the turret is possible, code in when range of motion is known (if (out of bounds) return -1;
        if (angle > Math.toRadians(40) || angle < 0) return -1; // if the angle is greater than 40 degrees, or less than zero, stop and return -1,

        //iterate and save
        BigDecimal targetPos = elevationCalculationIteration(angle); // in radians
        //convert the target pos to a value in encoder counts
        targetPos = targetPos.divide(BigDecimal.valueOf(robot.GO_BILDA_RADIANS_PER_COUNTS), elevationMC); // in encoder counts

        robot.runMotorToPosition(robot.turretElevator, targetPos.intValue(), 0.5);
        return 0;
    }
    /**
     * this function uses iteration to find the optimal position to move the elevator motor to in order to elevate the turret to a given angle,
     * this function works best when working with degrees so it takes an input and gives an output in radians, while using degrees for internal calculations
     * @param targetAngle the angle, in radians
     * @return the angle to move the elevator to, in radians
     */
    public BigDecimal elevationCalculationIteration (double targetAngle) {
        while (elevationGuessOffsetCalculation(-targetAngle, Math.toRadians(-elevationLastGuessDegrees.doubleValue())) > 0) { //convert the target angle, and the guess, to radians in the call to the elevationGuessOffsetCalculation() method, this is because trig in java is done in radians
            elevationLastGuessDegrees = elevationLastGuessDegrees.add(BigDecimal.valueOf(elevationStep), elevationMC);
        }
        elevationLastGuessDegrees = elevationLastGuessDegrees.subtract(BigDecimal.valueOf(elevationStep), elevationMC);

        if (!(Math.abs(elevationGuessOffset) < 0.00001 || elevationStep < 0.0001)) { //skip iteration if the offset is close enough to zero, or it has iterated past 4 decimal places
            elevationStep /= 10;
            elevationCalculationIteration(targetAngle);
        }

        return (elevationLastGuessDegrees.multiply(BigDecimal.valueOf(Math.PI), elevationMC)).divide(BigDecimal.valueOf(180), elevationMC);
    }
    public double elevationGuessOffsetCalculation (double targetAngle, double currentGuess) {
        double x1 = Math.sin(targetAngle) * .02;
        double y1 = Math.cos(targetAngle) * .02;
        double x2 = Math.sin(currentGuess) * .02 + 0.0762;
        double y2 = Math.cos(currentGuess) * .02;
        double x3 = Math.cos(currentGuess) * .09525 + x2;
        double y3 = -Math.sin(currentGuess) * .09525 + y2;

        elevationGuessOffset = (y1 / x1) + ((x1-x3) / (y1-y3));
        return elevationGuessOffset;
    }
    public void fireTurret() {
        //stop robot movement
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        //get heading and pitch (skip for now, probably not needed bc alot of what I would put here is redundant (already happens in the teleop))
        posTarMan.update(robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
        aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());

        //move turret to aim at target
        if (rotateTurretTo(aimMan.getHeadingToTarget()) == -1) rotateTurretTo(Math.toRadians(0)); //try to aim to whatever the aim manager thinks it needs to, if that doesn't work, point forward
        if (elevateTurretTo(aimMan.getPitchToTarget()) == -1)  rotateTurretTo(Math.toRadians(40));//same as above, just pitch instead of heading

        //spin up flywheels and wait a bit to let everything move up to speed, the flywheels are not the same speed in order to create a spin
        robot.flyWheel1.setPower(0.9);
        robot.flyWheel2.setPower(1.0);

        sleep(500);

        //launch ring.
        // rotate the launch servo enough that the ring gets pushed into the flywheels, and the launcher is ready to accept the next ring
        robot.turretLauncher.setPower(-1);
        sleep(500); ///this number will change with testing

        //reset/prep other components for next shot
        robot.flyWheel1.setPower(0);
        robot.flyWheel2.setPower(0);
        elevateTurretTo(0);
        rotateTurretTo(0);
        robot.turretLauncher.setPower(0.5);
        loaded = false;
    }
    /**
     * this will reload a ring if needed, or clear the turret otherwise (if there was a jam for instance)
     */
    public void reloadTurret() {
        //make sure turret is aligned
        rotateTurretTo(0);
        elevateTurretTo(0);
        if (loaded) { //if loaded, unload
            //set the conveyors to reverse
            robot.conveyor.setPower(-1);
            //clear turret
            robot.turretLauncher.setPower(.5);
            sleep(500);
            //stop conveyors
            robot.conveyor.setPower(0);
            loaded = false;
        } else { //if unloaded, load
            //set the conveyors to forward
            robot.conveyor.setPower(1);
            elevateTurretTo(Math.toRadians(15)); //elevate the turret slightly to assist with the reload
            //wiggle the launching thing around a bit
            robot.turretLauncher.setPower(-0.3);
            robot.conveyor.setPower(0);
            sleep(300);
            robot.turretLauncher.setPower(0);
            sleep(100);
            robot.turretLauncher.setPower(-.5);
            sleep(300); //adjust timing
            robot.turretLauncher.setPower(0.15);
            elevateTurretTo(0);
            loaded = true;
        }
    }
}
