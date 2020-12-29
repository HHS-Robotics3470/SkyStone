package org.firstinspires.ftc.teamcode;

import android.service.autofill.ImageTransformation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
debugging things will be on the d-pad, controls on the a,b,x,y buttons
 */
//TODO: 12/21/2020 TODO LIST: test code for the goal grabber, test code conveyor belt, code reloading sequence, code turret elevation adjustment, code turret heading adjustment


//TODO: 10/21/2020 eventually, change this from a testbed type thing (with all the telemetry), to a final product
@TeleOp(name="TeleOP Test Bed Ultimate Goal", group="UltimateGoal")
public class AdvancedTestBedTeleopUltimateGoal extends LinearOpMode {
    /* Declare OpMode members. initialize some classes */
    //TODO: remove "testing" from HardwareUltimateGoal Initializer, and uncomment the bit in line 14
    HardwareUltimateGoal robot          = new HardwareUltimateGoal("testing");
    PositionAndTargetManager posTarMan  = new PositionAndTargetManager(robot, HardwareUltimateGoal.readPosition(), HardwareUltimateGoal.readHeading(), true); //TODO: 10/21/2020 change true to false if on team blue
    ElapsedTime runtime                 = new ElapsedTime();
    AimAssist aimMan                    = new AimAssist(robot, posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.bestTargetPosition(0));

    double currentTurretHeading; //initialize after initializing robot hardware
    double currentTurretPitch   = 0;

    double cooldownLength = 500;
    double startOfCooldown = -cooldownLength; //so that normal telemetry starts immediately

    @Override
    public void runOpMode()
    {
        // declare some variables if needed
        int leftCounts  = 0;
        int rightCounts = 0;

        boolean abort = false; //variable to control whether or not it allows movement commands to go through

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
        while (opModeIsActive())
        {

          // controls and movement
            tankControls(gamepad1.right_stick_y, gamepad1.left_stick_y);

          // update position and aim managers
            leftCounts     = robot.leftDrive.getCurrentPosition(); //total left encoder counts
            rightCounts    = robot.rightDrive.getCurrentPosition();//total right encoder counts
            posTarMan.update(leftCounts, rightCounts);
            aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());


          // automated movement (turret), and fire?, and other controls
            /* for controls: (as of 12/26/2020)
            a,b,x,y pad will have turret controls

            bumpers and triggers will be debug things

            thumbsticks will control movement

            d-pad will control the intake and wobble grabber (up and down, move intake up/down; right, toggle conveyor; left, toggle grabber
             */

            //change targets, and name the new target
            if (gamepad1.x && !abort) { // if driver presses X, change targets
                posTarMan.bestTargetPosition(runtime.seconds());
                telemetry.addLine("TARGET CHANGED");
                telemetry.addData("new target", posTarMan.getCurrentTarget());
                telemetry.update();
                startOfCooldown = getRuntime();
            }
            //fire turret
            if(gamepad1.a)          fireTurret();

            //shows info about the motor encoders
            if (gamepad1.left_trigger > 0.5) {
                telemetry.addLine("motor encoder counts");
                telemetry.addData("left motor", robot.leftDrive.getCurrentPosition());
                telemetry.addData("right motor", robot.rightDrive.getCurrentPosition());
                telemetry.update();
                startOfCooldown = getRuntime();
            }
            //abort button
            if (gamepad1.left_bumper) abort = !abort;

            //d up and down, move intake up/down
            if (gamepad1.dpad_up && robot.intakePulley.getPower() <= 1)     robot.intakePulley.setPower(robot.intakePulley.getPower() + 1);
            if (gamepad1.dpad_down && robot.intakePulley.getPower() >=-1)   robot.intakePulley.setPower(robot.intakePulley.getPower() - 1);
            //d right, toggle conveyor
            if (gamepad1.dpad_right)                                        robot.conveyor.setPower(-1 * (robot.conveyor.getPower() - 0.5) + 0.5);
                                                                            //power should only ever be 0 or 1; -(0 - 0.5) + 0.5 = 1; -(1 - 0.5) + 0.5 = 0
            //d left, toggle grabber
            if (gamepad1.dpad_left)                                         robot.wobbleGrabber.setPosition(-1 * (robot.wobbleGrabber.getPosition() - 0.5) + 0.5);
                                                                            //position should only ever be 0 or 1, same deal as before

            //telemetry
            basicTelemetryManager();
        }

        //after opMode, save current position and heading for reasons
        //HardwareUltimateGoal.writePositionHeading(posTarMan.getRobotPosition(), posTarMan.getRobotHeading());
    }

    public void basicTelemetryManager() {
        if (getRuntime() >= startOfCooldown + cooldownLength) {
            telemetry.addLine("position information");
            telemetry.addData("x"       , posTarMan.getRobotPosition()[0]);
            telemetry.addData("y"       , posTarMan.getRobotPosition()[1]);
            telemetry.addData("heading" , Math.toDegrees(posTarMan.getRobotHeading()));

            telemetry.addLine("turret information");
            telemetry.addData("heading"             , currentTurretHeading);
            telemetry.addData("pitch"               , currentTurretPitch);
            telemetry.addData("heading to target"   , aimMan.getHeadingToTarget());
            telemetry.addData("pitch to target"     , aimMan.getPitchToTarget());

            telemetry.addLine("target info");
            telemetry.addData("target"  , posTarMan.getCurrentTarget());
            telemetry.addData("x"       , posTarMan.getTargetPosition()[0]);
            telemetry.addData("y"       , posTarMan.getTargetPosition()[1]);
            telemetry.addData("z"       , posTarMan.getTargetPosition()[2]);

            telemetry.update();
        }
    }


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
        robot.leftDrive.setPower(-left);
        robot.rightDrive.setPower(-right);
    }

    //TODO 12/25/2020 this should still work, as the mechanism to rotate the turret is still fundamentally the same, however it needs to be tested
    /**
     * given the angle relative to the field, convert to the angle relative to the robot (front = 0)
     * @param angle angle relative to field
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and rotates the turret to the needed position
     */
    public int rotateTurretTo(double angle) {
        double pos = robot.turretRotator.getCurrentPosition();
        double targetPosition;
        double robotHeading = posTarMan.getRobotHeading();

        //heading of turret relative to robot (convert encoder count of rotator to radians
        currentTurretHeading = pos * robot.CORE_HEX_RADIANS_PER_COUNTS;
        //heading of turret relative to field (with robot pov (angle 0 = parallel to x-axis, just like the angle being provided)
        currentTurretHeading = currentTurretHeading + robotHeading; // turret heading + robot heading

        //make sure headings are in the range [-pi,pi] instead of [0,2pi]


        //find the position that the turret needs to rotate to
        targetPosition = angle - currentTurretHeading; //in radians
        targetPosition /= robot.CORE_HEX_RADIANS_PER_COUNTS; //in encoder counts

        //check if it's in the deadzone (range of motion: 120degrees, so +- pi/3 radians from the zero of the turret, the /3 represents the dead zone btw)
        if ( (pos + targetPosition) > robot.CORE_HEX_COUNTS_PER_MOTOR_REV / 2 / 3 || (pos + targetPosition) < - robot.CORE_HEX_COUNTS_PER_MOTOR_REV / 2 / 3) {
            return -1;
        }
        robot.runMotorToPosition(robot.turretRotator,(int) (pos + targetPosition), 1);
        return 0;
    }
    //TODO 12/25/2020 update this class, need to do some math to figure out how rotating the motor translates to elevation
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
        //logic statement to make sure that the given target angle of the turret is possible, code in when range of motion is known (if (out of bounds) return -1;

        //iterate and save
        double targetPosRad = elevationCalculationIteration(angle);
        //reset the global values that control the iteration
        elevationStep = 1;
        elevationLastGuess = 0;
        elevationGuessOffset = 1;
        return 0;
    }
    double elevationStep = 1;
    double elevationLastGuess = 0;
    double elevationGuessOffset = 1;
    public double elevationCalculationIteration (double targetAngle) {
        while (elevationGuessOffsetCalculation(-targetAngle, elevationLastGuess) > 0) {
            elevationLastGuess += elevationStep;
        }
        //check if end condition is met, if so, return the angle, if not, modify step and iterate
        if (Math.abs(elevationGuessOffset) > 0.000001) { // if the offset is close enough to zero
            return elevationLastGuess;
        } else {
            elevationStep /= 10;
            elevationCalculationIteration(targetAngle);
            return -1;
        }
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


    boolean firingError = false;
    public void fireTurret() {
        //steps: stop movement of the motors, get heading and pitch, (make sure target is in range), move to that heading and pitch (spin up the flywheel), launch

        //stop robot movement
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //get heading and pitch (skip for now, probably not needed bc alot of what I would put here is redundant (already happens in the teleop))
        posTarMan.update(robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
        aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());

        switch ((int) aimMan.getPitchToTarget()) {
            case -1://target out of range
                telemetry.addLine("target out of range, move closer or change targets");
                telemetry.addData("currently targeting", posTarMan.getCurrentTarget());
                startOfCooldown = getRuntime();
                break;
            case -2://target would require going over range/height cap
                telemetry.addLine("hitting the current target would require going over the range / height cap");
                telemetry.addData("currently targeting", posTarMan.getCurrentTarget());
                startOfCooldown = getRuntime();
                break;
            default: //continue as normal

                if (!firingError) { // if there wasn't a firing error last attempt (there is not a ring already in launch position), put a ring in launch position
                    //rotate the launch servo enough to lock the ring into place before aiming
                    robot.turretLauncher.setPower(1);
                    sleep(robot.launcherTimeToRotate / 8); //subject to chance, but should be close enough
                    robot.turretLauncher.setPower(-1);
                    sleep(robot.launcherTimeToRotate / 16); //servo is now at 1/16th a rotation

                    // rotate the launch servo enough so that the pusher is at the back of the ring, ready to push it into the flywheels, and holding it in place
                    robot.turretLauncher.setPower(1);
                    sleep(robot.launcherTimeToRotate * 15 / 16); ///this number WILL change with testing
                    robot.turretLauncher.setPower(0); //has now done one full rotation in total, securing a ring in the process
                } // if there was a firing error, it'll simply skip, and try to aim again

                //move turret to aim at target
                if (rotateTurretTo(aimMan.getHeadingToTarget()) == -1) { //executes the rotation method, and if there is an error, runs the body of the IF statement, delcaring an error, and aborting launch
                    //the target is in deadzone
                    telemetry.addLine("target is in turret dead zone, try rotating the robot");
                    startOfCooldown = getRuntime();
                    firingError = true;
                } else firingError = false;

                if (elevateTurretTo(aimMan.getPitchToTarget()) == -1) { //same deal as before, just applied to the elevator
                    //the target is in the deadzone
                    telemetry.addLine("target is in elevator deadzone, try moving the robot closer");
                    startOfCooldown = getRuntime();
                    firingError = true;
                } else firingError = false;

                if (!firingError) {
                    //TODO 12/25/2020 this should work, but i need to figure out what launcherTimeToRotate should be, and i'll need a actual continuous servo

                    //spin up flywheels and wait a bit to let everything move up to speed
                    robot.flyWheel1.setPower(0.9);
                    robot.flyWheel2.setPower(1.0);

                    sleep(500);

                    //launch ring, and go through reload sequence
                    // rotate the launch servo enough that the ring gets pushed into the flywheels, and the launcher is ready to accept the next ring
                    robot.turretLauncher.setPower(1);
                    sleep(robot.launcherTimeToRotate * 1); ///this number will change with testing
                    robot.turretLauncher.setPower(0);

                    //reset/prep other components for next shot
                    elevateTurretTo(0);
                    rotateTurretTo(0);

                    robot.flyWheel1.setPower(0);
                    robot.flyWheel2.setPower(0);
                }

        }
    }
}

