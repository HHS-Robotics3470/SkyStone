package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.math.BigDecimal;
import java.math.MathContext;

/**
 * basic autonomous, will probably be dead reckoning.
 *
 * after every step of the autonomous, make a call to update the position manager
 */

//TODO: Rewrite for 2 things: 1) distance sensor, 2) odometry

@Autonomous(name="Autonomous Ultimate Goal A", group="UltimateGoal")
public class AutonomousUltimateGoal extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareUltimateGoal robot          = new HardwareUltimateGoal("testing");
    PositionAndTargetManager posTarMan  = new PositionAndTargetManager(robot, new double[]{1.79705 - 0.8, -1.79705 + 0.225}, -Math.PI/2 , true); //TODO: 10/21/2020 change true to false if on team blue
    ElapsedTime runtime                 = new ElapsedTime();
    AimAssist aimMan                    = new AimAssist(robot, posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.bestTargetPosition(0));

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        int timeToLowerIntake = 1000; //needs testing

        //set up other things
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sounds stuff
        int lessGoSoundID = hardwareMap.appContext.getResources().getIdentifier("lessgo","raw",hardwareMap.appContext.getPackageName());
        boolean lessGoFound = false;
        if (lessGoSoundID != 0) lessGoFound= SoundPlayer.getInstance().preload(hardwareMap.appContext, lessGoSoundID);

        int notGoodSoundID = hardwareMap.appContext.getResources().getIdentifier("ohshitnotgood.wav","raw",hardwareMap.appContext.getPackageName());
        boolean notGoodFound = false;
        if (notGoodSoundID!=0) notGoodFound=SoundPlayer.getInstance().preload(hardwareMap.appContext,notGoodSoundID);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //sounds stuff
        if (lessGoFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext,lessGoSoundID);

        // Step through each leg of the path,
        //TODO: need to recode the auto-routine
        //---------------------------------------------------------------------------------------\\

        // PARAMETER VALUES ARE TENTATIVE AS EXPERIMENTATION IS NEEDED TO FINE TUNE AND ADJUST THESE VALUES \\
        posTarMan.setTarget(4); //set the target to be the the mid goal

        //initialization / startup stuff
        robot.distanceServo.setPosition(1);
        robot.wobbleGrabber.setPosition(0); // grab the wobble goal securely

        robot.intakePulley.setPower(-1);

        sleep(timeToLowerIntake / 6);

        robot.intakePulley.setPower(0);

        rotateTurretTo(Math.toRadians(40));
        rotateTurretTo(Math.PI/6); // the turret starts 30 degrees off center, move back to the middle
        robot.turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.intakePulley.setPower(-1); //lower the intake 1/2 of the way
        sleep(timeToLowerIntake / 6);    //POTENTIAL ISSUE, not waiting, the pulley never raises

        robot.intakePulley.setPower(0);

        //------------------------------STEP 1: scan ring stack, and get to position (to start depositing the wobble goal)------------------------------//
        //reverse to the stack
        encoderDrive(robot.leftDrive,robot.rightDrive,robot.leftOdometry,robot.rightOdometry,robot.horizOdometry, 0.69,-1);
        //position should now be +0.69y from start position


        //scan with sensor
        //scan the stack size
        short targetZone = 0; //0 == 0 rings, 1 == 1 ring, 2 == 4 rings
        if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 10) targetZone = 2;
        if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 5) targetZone = 1;

        //turn ccw 45 degrees
        encoderTurn(robot.leftDrive,robot.rightDrive,robot.leftOdometry,robot.rightOdometry,robot.horizOdometry, Math.toRadians(45), -1);

        //move past rings
        encoderDrive(robot.leftDrive,robot.rightDrive,robot.leftOdometry,robot.rightOdometry,robot.horizOdometry, 0.35, -1);

        //rotate cw 45 degrees
        encoderTurn(robot.leftDrive,robot.rightDrive,robot.leftOdometry,robot.rightOdometry,robot.horizOdometry, Math.toRadians(45), 1);

        //move to launch line
        encoderDrive(robot.leftDrive,robot.rightDrive,robot.leftOdometry,robot.rightOdometry,robot.horizOdometry, 0.6, -1);

        //------------------------------STEP 2: branches for different target zones------------------------------//
        if (targetZone == 0) {
            //deposit, move to launch position path 1

        } else {
            //move to zone c

            if (targetZone == 4) {
                //deposit, move back, move to launch position path 1
            } else {
                //turn and move to b, deposit, continue to launch position
            }
        }

        //------------------------------STEP 3: aim and fire :D ------------------------------//




        //------------------------------STEP 4: aim and fire twice, then park on the launch line------------------------------//




        /*
        //move forward to the target zone goal (closest to start position) distance is ~160 centimeters
        encoderDrive(robot.leftDrive,robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, 1.6 * .8125, -.75);
        //encoderDrive(robot.leftDrive,robot.rightDrive, .3 * .8125, -.5);

        // drop the wobble used to be here

        encoderDrive(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, .2 * .8125,1);

        //turn left, so that the front of the robot (turret side) is facing to closest wall
        //used to be 65, changed to 55, changed to 80
        encoderTurn(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, Math.toRadians(80 * .8125), .75); //if it turns the wrong way, multiply the angle by -1
        //supposed to be 180

        //move backwards until the robot is at the firing area (where gabe goes to shoot) (4 feet) 48 * 2.54 / 100 = 1.2192
        encoderDrive(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, .4 * .8125,-1);

        //release the wobble goal, and fully lower the intake
        robot.wobbleGrabber.setPosition(0.8);
        //robot.intakePulley.setPower(-1);
        //sleep(timeToLowerIntake * 3/4);
        //robot.intakePulley.setPower(0);

        /*
        //angle the robot a bit toward the goals,
        encoderTurn(robot.leftDrive, robot.rightDrive, -Math.PI/4, 1); //if it turns the wrong way, multiply the angle by -1

        //move back a bit to ensure it's in the launch zone
        encoderDrive(robot.leftDrive, robot.rightDrive, 0.25, 1);
         */
        /*
        //fire twice:
        //aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());
        //  fire loaded ring
        //33 hit a powershot
        //27 hits the goals
        fireTurret(Math.toRadians(27), Math.toRadians(25));

        robot.intakePulley.setPower(-1);
        sleep(timeToLowerIntake * 2/3);

        robot.intakePulley.setPower(0);


        //  reload and run conveyor a bit more if needed (flipped order)
        rotateTurretTo(0);
        elevateTurretTo(Math.toRadians(15));
        robot.turretLauncher.setPower(.5);

        robot.conveyor.setPower(1);
        sleep(500);
        robot.conveyor.setPower(0);
        reloadTurret();

        //  fire again
        fireTurret(Math.toRadians(25), Math.toRadians(25));


        //turn a bit and park over the launch line
        //encoderTurn(robot.leftDrive, robot.rightDrive, - Math.toRadians(10 * .8125), 1);
        encoderDrive(robot.leftDrive, robot.rightDrive, robot.leftOdometry, robot.rightOdometry, robot.horizOdometry, .1, 1);



        robot.turretLauncher.setPower(0.5); */
        rotateTurretTo(0);
        //Saves the robot's position and heading
        HardwareUltimateGoal.writePositionHeading(posTarMan.getRobotPosition(), posTarMan.getRobotHeading());
    }
    //TODO: eventually make a program that lets the robot move in arcs or something
    /**
     * drives the robot forward a given distance, to go in reverse, give a negative value for power
     * @param left the motor on the left
     * @param right the motor on the right
     * @param horizOdo the horizontal odometry encoder
     * @param distance the distance the robot should move, meters, always positive, IMPORTANT, You're passing the desired change, not a desired place (this makes more sense with the turn method)
     * @param power the power the robot should move at
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
        int deltaLeft  = (leftOdo.getCurrentPosition()  * robot.getLeftDirection())  - initLeft;
        int deltaRight = (rightOdo.getCurrentPosition() * robot.getRightDirection()) - initRight;
        int deltaHoriz = horizOdo.getCurrentPosition()*robot.getHorizDirection() - initHoriz;
        //while the absolute value of (average side change) - (desired change) is not less than or equal to (is greater than) allowed side count offset:
        while ( Math.abs( Math.abs((deltaLeft - deltaRight)/2) - (int)distance ) > robot.getSideOdoAllowedCountOffset() ) {
            posTarMan.update(leftOdo.getCurrentPosition(), rightOdo.getCurrentPosition(), horizOdo.getCurrentPosition());
            //update delta__'s
            deltaLeft  = (leftOdo.getCurrentPosition()  * robot.getLeftDirection())  - initLeft;
            deltaRight = (rightOdo.getCurrentPosition() * robot.getRightDirection()) - initRight;
            deltaHoriz = (horizOdo.getCurrentPosition() * robot.getHorizDirection()) - initHoriz;


            telemetry.addData("left encoder count", robot.leftOdometry.getCurrentPosition()*robot.getLeftDirection());
            telemetry.addData("right encoder count", robot.rightOdometry.getCurrentPosition()*robot.getRightDirection());
            telemetry.addData("horizontal encoder count", robot.horizOdometry.getCurrentPosition()*robot.getHorizDirection());
            telemetry.update();


            //move
            //make sure it's not drifting
            //if there is a noticable difference in the distance travelled by each side OR the horiz encoder detects too much of a change in angle
            if ((Math.abs(deltaLeft - deltaRight) > robot.getSideOdoAllowedCountOffset())){// ||/*&&*/ (deltaHoriz > robot.getHorizOdoAllowedCountOffset())) {
                if (deltaLeft>deltaRight) {left.setPower(power*0.95);right.setPower(power);}         //if left has gone further, make it go slower
                else if (deltaLeft<deltaRight) {left.setPower(power*0.95);right.setPower(power);}    //if right has gone further, make it go slower
            } else {left.setPower(power);right.setPower(power);}
        }

        left.setPower(0);
        right.setPower(0);

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
            }*/
        /*
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
        // old from pre-odo times
        double wheelCircumference = robot.ODOMETRY_WHEEL_DIAMETER_METERS * Math.PI;
        double turnCircumference = robot.getRobotOdometryWidth() * Math.PI;
        double angleCircumference = angle * robot.getRobotOdometryLength();//in meters //arc length formula
        angleCircumference *= robot.ODOMETRY_COUNTS_PER_METER; // in encoder ticks

        int initHorizCount = horizOdo.getCurrentPosition() * robot.getHorizDirection();
        // target heading
        double targetHeading = posTarMan.getRobotHeading();
        if (power > 0) targetHeading += angle;
        else targetHeading -= angle;

        //TODO: maybe remove this ??
        /*orientate the target heading
        double twoPI = 2 * Math.PI;
        if (targetHeading > Math.PI) {
            targetHeading = -twoPI + targetHeading;
        } else if (targetHeading < -Math.PI) {
            targetHeading = twoPI + targetHeading;
        }*/

        left.setPower(-power);
        right.setPower(power);
        //TODO: after testing the position manager, if it is what's not working (and i can't fix it), change this to use encoders to guide it rather than the position tracker
        //while the current heading is too far from the target heading, move
        while ( Math.abs(targetHeading - posTarMan.getRobotHeading()) > 0.017/*robot.getHorizOdoAllowedCountOffset()*/ /*&& Math.abs(angleCircumference - (horizOdo.getCurrentPosition()-initHorizCount)) > robot.getHorizOdoAllowedOffset*/) {
            posTarMan.update(leftOdo.getCurrentPosition(), rightOdo.getCurrentPosition(), horizOdo.getCurrentPosition());

            telemetry.addData("left encoder count", robot.leftOdometry.getCurrentPosition()*robot.getLeftDirection());
            telemetry.addData("right encoder count", robot.rightOdometry.getCurrentPosition()*robot.getRightDirection());
            telemetry.addData("horizontal encoder count", robot.horizOdometry.getCurrentPosition()*robot.getHorizDirection());
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
    public int rotateTurretTo(double angle, double robotHeading) {
        double targetPosition;

        //heading relative to field -> heading relative to the robot
        targetPosition = angle - robotHeading;
        //check if heading rel. to robot is in the deadzone, if so, return -1
        if (targetPosition >= Math.toRadians(90) || targetPosition < -Math.toRadians(45)) {rotateTurretTo(0); return -1;}
        //convert the heading rel. to robot into the needed encoder counts
        targetPosition /= robot.CORE_HEX_RADIANS_PER_COUNTS;
        //make sure that the robot rotates the best direction to reach goal

        //rotate to that position and return 0
        robot.runMotorToPosition(robot.turretRotator, (int) targetPosition, 0.5);
        return 0;
    }
    /**
     * modification of the other rotateTurret method, this one disregards the robots heading
     * given the angle relative to the robot, move the turret to that angle
     * @param angle angle relative to robot
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and rotates the turret to the needed position
     */
    public int rotateTurretTo(double angle) {
        double targetPosition;

        //heading relative to field -> heading relative to the robot
        targetPosition = angle;
        //check if heading rel. to robot is in the deadzone, if so, return -1
        if (targetPosition >= Math.toRadians(90) || targetPosition < -Math.toRadians(45)) {rotateTurretTo(0); return -1;}
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

    /**
     * fires with given inputs
     */
    public void fireTurret(double heading, double elevation) {
        //stop robot movement
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        //get heading and pitch (skip for now, probably not needed bc alot of what I would put here is redundant (already happens in the teleop))
        //posTarMan.update(robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
        //aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());


        //move turret to aim at target
        rotateTurretTo(heading);
        elevateTurretTo(elevation);

        sleep(100);

        //spin up flywheels and wait a bit to let everything move up to speed, the flywheels are not the same speed in order to create a spin
        robot.flyWheel1.setPower(0.9);
        robot.flyWheel2.setPower(1.0);

        sleep(1000);

        //launch ring.
        // rotate the launch servo enough that the ring gets pushed into the flywheels, and the launcher is ready to accept the next ring
        robot.turretLauncher.setPower(-1);

        sleep(500); ///this number will change with testing

        //reset/prep other components for next shot
        rotateTurretTo(0);
        robot.flyWheel1.setPower(0);
        robot.flyWheel2.setPower(0);
        elevateTurretTo(0);
        robot.turretLauncher.setPower(0.5);
        loaded = false;
    }

    /**
     * fires with the aim and position managers
     */
    public void fireTurret() {
        //stop robot movement
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        //get heading and pitch (skip for now, probably not needed bc alot of what I would put here is redundant (already happens in the teleop))
        posTarMan.update(robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition(), robot.horizOdometry.getCurrentPosition());
        aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());

        //move turret to aim at target
        if (rotateTurretTo(aimMan.getHeadingToTarget()) == -1) rotateTurretTo(Math.toRadians(0)); //try to aim to whatever the aim manager thinks it needs to, if that doesn't work, point forward
        if (elevateTurretTo(aimMan.getPitchToTarget()) == -1)  elevateTurretTo(Math.toRadians(40));//same as above, just pitch instead of heading



        //spin up flywheels and wait a bit to let everything move up to speed, the flywheels are not the same speed in order to create a spin
        robot.flyWheel1.setPower(0.9);
        robot.flyWheel2.setPower(1.0);

        sleep(500);

        //launch ring.
        // rotate the launch servo enough that the ring gets pushed into the flywheels, and the launcher is ready to accept the next ring
        robot.turretLauncher.setPower(-1); // if this moves the wrong way, set to 1
        sleep(HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE/4);

        //reset/prep other components for next shot
        robot.turretLauncher.setPower(1);
        sleep(HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE/4);
        /*old from when launcher was a standard servo// rotate the launch servo enough that the ring gets pushed into the flywheels, and the launcher is ready to accept the next ring
        robot.turretLauncher.setPower(-1);
        sleep(500); ///this number will change with testing*/

        //reset/prep other components for next shot
        robot.flyWheel1.setPower(0);
        robot.flyWheel2.setPower(0);
        elevateTurretTo(0);
        rotateTurretTo(0);
        robot.turretLauncher.setPower(0/*.5*/);
        loaded = false;
    }
    /**
     * this will reload a ring if needed, or clear the turret otherwise (if there was a jam for instance)
     */
    public void reloadTurret() {
        //make sure turret is aligned
        rotateTurretTo(0);
        elevateTurretTo(Math.toRadians(15));//elevate the turret slightly to assist with the reload

        if (loaded) { //if loaded, unload
            //set the conveyors to reverse
            robot.conveyor.setPower(-1);
            //rotate the launch servo one full backwards rotation
            robot.turretLauncher.setPower(1);
            sleep(HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE);
            //stop launcher, wait a bit, then stop conveyor
            robot.conveyor.setPower(0);
            sleep(500);
            robot.turretLauncher.setPower(0);

            /*old, from when the turret launcher was a normal servo
            //set the conveyors to reverse
            robot.conveyor.setPower(-1);
            //clear turret
            robot.turretLauncher.setPower(.5);
            sleep(500);
            //stop conveyors
            robot.conveyor.setPower(0);*/
            loaded = false;
        } else { //if unloaded, load
            //set the conveyors to forward
            robot.conveyor.setPower(1);

            //turn the launcher in one full rotation
            robot.turretLauncher.setPower(-1);
            sleep(HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE/4);

            //stop the launcher, this interruption should help with missaligned loads
            robot.turretLauncher.setPower(0);
            robot.conveyor.setPower(0);
            sleep(100);

            //finish the rotation
            robot.turretLauncher.setPower(-1);
            sleep((3*HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE)/4);

            //stop the launcher
            robot.turretLauncher.setPower(0);

            /* old from when turret was a standard servo
            //set the conveyors to forward
            robot.conveyor.setPower(1);
            sleep(300);

            //wiggle the launching thing around a bit
            robot.turretLauncher.setPower(-0.3);
            robot.conveyor.setPower(0);
            sleep(300);

            robot.turretLauncher.setPower(0);
            sleep(100);

            robot.turretLauncher.setPower(-.75);
            sleep(300); //adjust timing

            robot.turretLauncher.setPower(0.1);*/
            elevateTurretTo(0);
            loaded = true;
        }
    }
}
