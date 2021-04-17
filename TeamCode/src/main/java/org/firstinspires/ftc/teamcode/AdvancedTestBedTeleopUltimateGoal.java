package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.math.BigDecimal;
import java.math.MathContext;

/*
debugging things will be on the d-pad, controls on the a,b,x,y buttons
 */
/*TODO: 12/21/2020 TODO LIST: telemetry timer taking too long, or maybe not working
heading tracking is not working (should be fixed, needs testing
firing sequence may be having issues too (hold, may just be heading issue)
fixed: drives backwards (reverse motor direction, if that causes issues with the encoders, flip directions again, and )


auto: robot moved way to far forward, maybe the value for the encoder ticks per rotation is too high?
 */



//TODO: 10/21/2020 eventually, change this from a testbed type thing (with all the telemetry), to a final product
@TeleOp(name="TeleOP Test Bed Ultimate Goal", group="UltimateGoal")
public class AdvancedTestBedTeleopUltimateGoal extends LinearOpMode {
    /* Declare OpMode members. initialize some classes */
    //TODO: remove "testing" from HardwareUltimateGoal Initializer, and uncomment the bit in line 14
    HardwareUltimateGoal robot          = new HardwareUltimateGoal("testing");
    PositionAndTargetManager posTarMan  = new PositionAndTargetManager(robot, HardwareUltimateGoal.readPosition(), HardwareUltimateGoal.readHeading(), true); //TODO: 10/21/2020 change true to false if on team blue
    ElapsedTime runtime                 = new ElapsedTime();
    AimAssist aimMan                    = new AimAssist(robot, posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.bestTargetPosition(0));
    //variables for telemetry
    double currentTurretHeading = 0;
    double currentTurretPitch   = 0;
    double cooldownLength = 200;//duration that an error will display
    double startOfCooldown = -cooldownLength; //so that normal telemetry starts immediately
    //variables for the firing sequence, to prevent it from repeating steps unnecessarily
    boolean loaded = false;
    boolean firingError = false;
    //variables for the elevation iteration algorithm
    double elevationStep = 1;
    MathContext elevationMC = new MathContext(16);
    BigDecimal elevationLastGuessDegrees = new BigDecimal("00.0000", elevationMC);
    double elevationGuessOffset = 1;

    boolean abort = false; //variable to control whether or not it allows movement commands to go through

    //button locks
    boolean aButtonCurPressed;
    boolean aButtonPrevPressed=false;

    boolean bButtonCurPressed;
    boolean bButtonPrevPressed=false;

    boolean xButtonCurPressed;
    boolean xButtonPrevPressed=false;

    boolean leftBumperCurPressed;
    boolean leftBumperPrevPressed=false;

    boolean rightBumperCurPressed;
    boolean rightBumperPrevPressed=false;

    //for background changing
    View relativeLayout;

    @Override
    public void runOpMode()
    {
        //stuff for background changing
        //int relativeLayoutId=hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        //try {
            // declare some variables if needed
            int leftCounts = 0;
            int rightCounts = 0;
            int horizCounts = 0;

            /* Initialize the hardware variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // run until the end of the match (driver presses STOP)
            //maybe some other set up stuff depending on how we want to do this
            while (opModeIsActive()) {

                // controls and movement
                tankControls(gamepad1.right_stick_y, gamepad1.left_stick_y);

                // update position and aim managers
                leftCounts = robot.leftOdometry.getCurrentPosition(); //total left encoder counts
                rightCounts = robot.rightOdometry.getCurrentPosition();//total right encoder counts
                horizCounts = robot.horizOdometry.getCurrentPosition();
                posTarMan.update(leftCounts, rightCounts, horizCounts);
                aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());


                //controls
            /* for controls: (as of 12/26/2020)
            a,b,x,y pad will have turret controls
                a-fire (will reload if needed)
                b-reload
                x-change target
                y-make launcher rotate CW (direct to fire)      (in abort mode, makes the launcher rotate CCW)

            bumpers and triggers will be debug things
                left bumper - turn on abort mode
                right bumper - turn off abort mode
                left trigger - display encoder readings
                right trigger - sets the turrets zero position to wherever it's currently at

            thumbsticks will control movement

            d-pad will control the intake and wobble grabber (up and down, move intake up/down; right, toggle conveyor; left, toggle grabber
            */


                //////ABXY bindings/////
                //fire turret
                aButtonCurPressed = gamepad1.a;
                if (aButtonCurPressed && !aButtonPrevPressed) {
                    int oldTarget = posTarMan.getCurrentTarget();
                    fireTurret();
                    posTarMan.setTarget(oldTarget);
                }
                aButtonPrevPressed = aButtonCurPressed;
                //reload/clear
                bButtonCurPressed = gamepad1.b;
                if (bButtonCurPressed && !bButtonPrevPressed) {
                    reloadTurret();
                }
                bButtonPrevPressed = bButtonCurPressed;
                //change targets, and name the new target
                xButtonCurPressed = gamepad1.x;
                if (xButtonCurPressed && !xButtonPrevPressed) { // if driver presses X, change targets
                    posTarMan.bestTargetPosition(runtime.seconds());
                    telemetry.addLine("TARGET CHANGED");
                    telemetry.addData("new target", posTarMan.getCurrentTargetDesc());
                    telemetry.update();
                }
                xButtonPrevPressed = xButtonCurPressed;
                //rotate turret launcher
                if (gamepad1.y) {
                    if (!abort) robot.turretLauncher.setPower(-0.5);
                    else robot.turretLauncher.setPower(0.5);
                } else robot.turretLauncher.setPower(0);


                /////bumper and trigger bindings/////

                //abort buttons
                leftBumperCurPressed=gamepad1.left_bumper;
                if (leftBumperCurPressed && !leftBumperPrevPressed) {
                    abort = true;
                } // changes the d-pad to control the turrets movement, also turns off aimbot
                leftBumperPrevPressed=leftBumperCurPressed;
                rightBumperCurPressed=gamepad1.right_bumper;
                if (rightBumperCurPressed && !rightBumperPrevPressed) {
                    abort = false;
                }// changes the d-pad to control things with the intake, also turns on aimbot
                rightBumperPrevPressed=rightBumperCurPressed;
                //reset the turret rotators zero position
                if (gamepad1.right_trigger > 0.5) {
                    robot.turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.turretRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    currentTurretHeading=0;
                }
                //telemetry
                if (gamepad1.left_trigger > 0.5) { //shows info about the motor encoders
                    telemetry.addLine("odometry encoder counts");
                    telemetry.addData("left odometry", robot.leftOdometry.getCurrentPosition() * robot.getLeftDirection());
                    telemetry.addData("right odometry", robot.rightOdometry.getCurrentPosition() * robot.getRightDirection());
                    telemetry.addData("horizontal ododmetry", robot.horizOdometry.getCurrentPosition() * robot.getHorizDirection());

                    telemetry.addLine("motor encoder counts");
                    telemetry.addData("left motor", robot.leftDrive.getCurrentPosition());
                    telemetry.addData("right motor", robot.rightDrive.getCurrentPosition());
                    telemetry.update();
                } else {
                    basicTelemetryManager();
                }


                ///////dpad bindings//////

                //d up and down, move intake up/down (if not in abort mode, if in abort, elevate turret up/down
                if (gamepad1.dpad_up) {
                    if (!abort) {
                        robot.intakePulley.setPower(1);
                    } //in normal mode, move intake up
                    else
                        elevateTurretTo(Math.toRadians(currentTurretPitch + 10)); //in abort mode, mode turret up
                } else robot.intakePulley.setPower(0);
                if (gamepad1.dpad_down) {
                    if (!abort) {
                        robot.intakePulley.setPower(-1);
                    } //in normal mode, move intake down
                    else
                        elevateTurretTo(Math.toRadians(currentTurretPitch - 5)); //in abort mode, mode turret down
                } else robot.intakePulley.setPower(0);
                //d right and left, toggle conveyor and grabber respectively (in normal mode, in abort mode, rotate turret right / left)
                if (gamepad1.dpad_right) {
                    if (!abort) {
                        robot.conveyor.setPower(-1 * (robot.conveyor.getPower() - 0.5) + 0.5);
                        sleep(200);
                    } //power should only ever be 0 or 1; -(0 - 0.5) + 0.5 = 1; -(1 - 0.5) + 0.5 = 0
                    else
                        rotateTurretTo(Math.toRadians(currentTurretHeading + 5)); //in abort mode, rotate turret to the right
                }
                //d left, toggle grabber
                if (gamepad1.dpad_left) {
                    if (!abort) {
                        robot.wobbleGrabber.setPosition(-1 * (robot.wobbleGrabber.getPosition() - 0.4) + 0.4);
                        sleep(200);
                    }//position should only ever be 0 or 0.8, same deal as before
                    else
                        rotateTurretTo(Math.toRadians(currentTurretHeading - 5)); //in abort mode, rotate turret to the left
                }

                /*
                //set the background color depending on whether or not the robot is in abort mode
                if (abort) relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.RED);
                    }
                });
                else relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.BLUE);
                    }
                });*/


            }
        /*} finally {
            //restore the background color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });*/
            //after opMode, save current position and heading for reasons
            //HardwareUltimateGoal.writePositionHeading(posTarMan.getRobotPosition(), posTarMan.getRobotHeading());
        //}
    }

    public void basicTelemetryManager() {

        telemetry.addData("in abort mode?", abort);
        telemetry.addLine("position information");
        telemetry.addData("x"       , posTarMan.getRobotPosition()[0]);
        telemetry.addData("y"       , posTarMan.getRobotPosition()[1]);
        telemetry.addData("heading" , Math.toDegrees(posTarMan.getRobotHeading()%(Math.PI*2)));//posTarMan.getRobotHeading());

        telemetry.addLine("turret information");
        telemetry.addData("heading"             , currentTurretHeading);
        telemetry.addData("pitch"               , currentTurretPitch);
        telemetry.addData("target"              , posTarMan.getCurrentTargetDesc());
        telemetry.addData("heading to target"   , aimMan.getHeadingToTarget());//Math.toDegrees(aimMan.getHeadingToTarget()));
        telemetry.addData("pitch to target"     , aimMan.getPitchToTarget());//Math.toDegrees(aimMan.getPitchToTarget()));

        telemetry.addLine("target info");
        telemetry.addData("target"  , posTarMan.getCurrentTargetDesc());
        telemetry.addData("x"       , posTarMan.getTargetPosition()[0]);
        telemetry.addData("y"       , posTarMan.getTargetPosition()[1]);
        telemetry.addData("z"       , posTarMan.getTargetPosition()[2]);

        telemetry.update();

    }

    //////////////////////////////robot control//////////////////////////////
    public void basicStickControls(double x, double y) {
        double leftPower  = 0;
        double rightPower = 0;
        if (y < -0.1 || y > 0.1){
            leftPower     = -y;
            rightPower    = -y;
        }
        if (x > 0.1) { //turn clockwise
            leftPower   += x;
            rightPower  -= x;
        } else if (x < -0.1){ //turn counter-clockwise
            leftPower   -= x;
            rightPower  +=x;
        }

        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
    }
    public void tankControls(double left, double right){
        //give left and right some increments
        int intLeft = (int)(left * 10.0);
        int intRight = (int)(right * 10.0);

        left = (double)intLeft/10.0;
        right = (double)intRight/10.0;

        robot.leftDrive.setPower(-left);
        robot.rightDrive.setPower(-right);
    }
    //////////////////////////////turret stuff//////////////////////////////
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
        if (targetPosition > Math.toRadians(45) || targetPosition < -Math.toRadians(45)) {rotateTurretTo(0); return -1;}
        currentTurretHeading = Math.toDegrees(targetPosition);
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
        if (targetPosition > Math.toRadians(60) || targetPosition < -Math.toRadians(45)) {rotateTurretTo(0); return -1;}
        currentTurretHeading = Math.toDegrees(targetPosition);
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
        /* basic layout
        -return -1 if something went wrong, or the desired angle is impossible
        -elevate to position
        -return 0, to show that nothing went wrong

        more detail
        -get the needed pitch to the target from the AimManager
        -calculate what position the elevation servo needs to rotate to in order the acheive that pitch (trig)
        -if that position is a deadzone, would be blocked by other parts of the robot, or is otherwise not safe to move toward, return -1, terminating this process
        -if that position is able to be rotated to, then do that, and return 0
         */
        /*
        2 options, find a function to calculate the required angle, or use iteration https://www.desmos.com/calculator/omowmwxgj2
         */
        //iteration
        //reset the global values that control the iteration
        elevationStep = 1;
        elevationLastGuessDegrees = BigDecimal.ZERO;
        elevationGuessOffset = 1;
        //logic statement to make sure that the given target angle of the turret is possible, code in when range of motion is known (if (out of bounds) return -1;
        if (angle > Math.toRadians(40) || angle < 0) {elevateTurretTo(0); return -1;} // if the angle is greater than 40 degrees, or less than zero, stop, reset and return -1,

        //iterate and save
        BigDecimal targetPos = elevationCalculationIteration(angle); // in radians
        currentTurretPitch = Math.toDegrees(angle);
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




    int curFireCycle = 0;
    public void fireTurret() {
        //stop robot movement
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        //get heading and pitch (skip for now, probably not needed bc alot of what I would put here is redundant (already happens in the teleop))
        posTarMan.update(robot.leftOdometry.getCurrentPosition(), robot.rightOdometry.getCurrentPosition(), robot.horizOdometry.getCurrentPosition());
        aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());

        switch ((int) aimMan.getPitchToTarget()) {
            case -1://target out of range
                telemetry.addLine("target out of range, move closer or change targets");
                telemetry.addData("currently targeting", posTarMan.getCurrentTargetDesc());
                if (!abort) telemetry.addLine("cycling targets");
                telemetry.update();
                sleep(100);
                //if not abort mode, cycle targets until one works
                if (!abort) {
                    if (curFireCycle<=5) {
                        posTarMan.cycleTarget();
                        curFireCycle++;
                    } else {
                        curFireCycle=0;
                    }
                    fireTurret();
                    break;
                }
            case -2://target would require going over range/height cap
                telemetry.addLine("hitting the current target would require going over the range / height cap");
                telemetry.addData("currently targeting", posTarMan.getCurrentTargetDesc());
                telemetry.update();
                sleep(100);
                if (!abort) break;
            default: //continue as normal
                if (!abort) {
                    if (!firingError && !loaded) { // if there wasn't a firing error last attempt (there is not a ring already in launch position), put a ring in launch position
                        reloadTurret();
                    } // if there was a firing error, it'll simply skip, and try to aim again

                    //move turret to aim at target
                    if (rotateTurretTo(aimMan.getHeadingToTarget()/*, posTarMan.getRobotHeading()*/) == -1) { //executes the rotation method, and if there is an error, runs the body of the IF statement, delcaring an error, and aborting launch
                        //the target is in deadzone
                        telemetry.addLine("target is in turret dead zone, try rotating the robot");
                        telemetry.update();
                        startOfCooldown = getRuntime();
                        firingError = true;
                        return;
                    } else firingError = false;

                    if (elevateTurretTo(aimMan.getPitchToTarget()) == -1) { //same deal as before, just applied to the elevator //commented out the abort thing so that it pitches for you no matter what
                        //the target is in the deadzone
                        telemetry.addLine("target is in elevator deadzone, try moving the robot closer");
                        telemetry.update();
                        startOfCooldown = getRuntime();
                        firingError = true;
                        return;
                    } else if (firingError)
                        elevateTurretTo(0); //this triggers if the turret couldn't rotate to the target, but could elevate, it just sets the turret to elevate back to zero
                    else firingError = false;
                }
                //wait for the turret to finish aiming (unneeded, the methods used to rotate the turrets already wait
                //while(robot.turretElevator.isBusy() || robot.turretRotator.isBusy()) sleep(10);

                if ((!firingError && loaded) || abort) {
                    //spin up flywheels and wait a bit to let everything move up to speed, the flywheels are not the same speed in order to create a spin
                    robot.flyWheel1.setPower(0.9);
                    robot.flyWheel2.setPower(1.0);

                    sleep(1000);

                    //launch ring.
                    // rotate the launch servo enough that the ring gets pushed into the flywheels, and the launcher is ready to accept the next ring
                    robot.turretLauncher.setPower(-1); // if this moves the wrong way, set to 1
                    sleep(HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE/4);

                    //reset/prep other components for next shot
                    robot.turretLauncher.setPower(1);
                    sleep(HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE/4);

                    robot.turretLauncher.setPower(0);
                    robot.flyWheel1.setPower(0);
                    robot.flyWheel2.setPower(0);
                    //elevateTurretTo(0);
                    //rotateTurretTo(0);
                    loaded = false;
                }
                break;
        }
    }

    /**
     * this will reload a ring if needed, or clear the turret otherwise (if there was a jam for instance)
     */
    public void reloadTurret() {
        //make sure turret is aligned
        robot.turretLauncher.setPower(.5);
        double prevHeading = currentTurretHeading;
        double prevPitch = currentTurretPitch;

        rotateTurretTo(0);
        elevateTurretTo(0);
        if (loaded) { //if loaded, unload //TODO: recode the unload sequence, when done, apply changes to the autonomous
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
        } else { //if unloaded, load //TODO: recode the reload sequence, when done, apply changes to the autonomous
            //set the conveyors to forward
            robot.conveyor.setPower(1);

            //turn the launcher in one full rotation
            robot.turretLauncher.setPower(-1);
            sleep(HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE/4);

            //stop the launcher, this interruption should help with missaligned loads
            robot.turretLauncher.setPower(0);
            sleep(100);

            //finish the rotation
            robot.turretLauncher.setPower(-1);
            sleep((3*HardwareUltimateGoal.LAUNCHER_TIME_TO_ROTATE)/4);

            //stop the launcher
            robot.turretLauncher.setPower(0);

            robot.conveyor.setPower(0);

            /*old, from when the turret launcher was a normal servo
            elevateTurretTo(Math.toRadians(15)); //elevate the turret slightly to assist with the reload
            //wiggle the launching thing around a bit
            robot.turretLauncher.setPower(-0.3);
            robot.conveyor.setPower(0);
            sleep(300);
            robot.turretLauncher.setPower(0);
            sleep(100);
            elevateTurretTo(0);
            robot.turretLauncher.setPower(-.75);
            sleep(300); //adjust timing
            robot.turretLauncher.setPower(0.1);*/
            loaded = true;
        }

        rotateTurretTo(Math.toRadians(prevHeading));
        elevateTurretTo(Math.toRadians(prevPitch));
    }
}

