package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
debugging things will be on the d-pad, controls on the a,b,x,y buttons
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
            //change targets, and name the new target
            if (gamepad1.x && !abort) { // if driver presses X, change targets
                posTarMan.bestTargetPosition(runtime.seconds());
                telemetry.addLine("TARGET CHANGED");
                telemetry.addData("new target", posTarMan.getCurrentTarget());
                telemetry.update();
                startOfCooldown = getRuntime();
            }

            //shows info about the motor encoders
            if (gamepad1.dpad_up) {
                telemetry.addLine("motor encoder counts");
                telemetry.addData("left motor", robot.leftDrive.getCurrentPosition());
                telemetry.addData("right motor", robot.rightDrive.getCurrentPosition());
                telemetry.update();
                startOfCooldown = getRuntime();
            }

            //abort button
            if (gamepad1.dpad_down) abort = !abort;
            //auto heading adjustment
            if (!abort)             rotateTurretTo(aimMan.headingToTarget);
            //fire turret
            if(gamepad1.a)          fireTurret();

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

        //check if it's in the deadzone (range of motion: 120degrees, so +- pi/3 radians from the zero of the turret)
        if ( (pos + targetPosition) > robot.CORE_HEX_COUNTS_PER_MOTOR_REV / 2 / 3 || (pos + targetPosition) < - robot.CORE_HEX_COUNTS_PER_MOTOR_REV / 2 / 3) {
            return -1;
        }
        robot.turretRotator.setTargetPosition((int) (pos + targetPosition));
        return 0;
        /* old code (for when it was using a servo instead of the core hex motor
        double pos = robot.turretRotator.getPosition(); //current position of the turret (from [0,1], representing [0,pi] degrees)

        double robotHeading = posTarMan.getRobotHeading(); //heading of robot
        if (robotHeading > Math.PI) {robotHeading -= 2*(robotHeading - Math.PI);}
        //assume that position 0 is pointing to the right of the robot

        //heading relative to robot
        currentTurretHeading = pos * Math.PI; // angle = position * range of motion
        //heading relative to field
        currentTurretHeading += robotHeading - Math.PI/2;

        //now, find the position the turret needs to go to, in order to point at the given angle
        double changeInAngle = angle - currentTurretHeading; //final angle - current angle = required change in angle
        pos += changeInAngle / Math.PI; // position + ( needed angle change / servo range of motion )

        //check if target angle is in a deadzone
        if (pos > 1 || pos < 0) {return -1;} //if target is in the deadzone, return -1
        else { //if target is reachable, rotate to the target and return 0
            robot.turretRotator.setPosition(pos);
            return 0;
        }
         */
    }
    /**
     * given the angle relative to the horizontal, move the turret to elevate to that pitch
     * @param angle desired angle of pitch
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and elevates the turret to the needed position
     */
    public double elevateTurretTo(double angle) {
        double deadzone = Math.PI/3; // angles above this constrain cannot be rotated to because of hardware constraints

        if (angle > deadzone) return -1;

        double pos;
        pos = angle / Math.PI; //angle / range of motion

        robot.turretElevator.setPosition(pos);
        return 0;
    }

    public void fireTurret() {
        //steps: stop movement of the motors, get heading and pitch, (make sure target is in range), move to that heading and pitch (spin up the flywheel), launch

        //stop robot movement
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        //get heading and pitch (skip for now, probably not needed bc alot of what I would put here is redundant (already happens in the teleop))
        posTarMan.update(robot.leftDrive.getCurrentPosition(), robot.rightDrive.getCurrentPosition());
        aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());


        //test if out of range
        /* coded as if statements
        if (aimMan.getPitchToTarget() == -1) { //target out of range
            telemetry.addLine("target out of range, move closer or change targets");
            telemetry.addData("currently targeting", posTarMan.getCurrentTarget());
        } else if (aimMan.getPitchToTarget() == -2) { //target would require going over range/height cap
            telemetry.addLine("hitting the current target would require going over the range / height cap");
            telemetry.addData("currently targeting", posTarMan.getCurrentTarget());
        } else { //continue as normal
            boolean error = false;
            //move turret to aim at target
            if (rotateTurretTo(aimMan.getHeadingToTarget()) == -1) {
                //the target is in deadzone
                telemetry.addLine("target is in turret dead zone, try rotating the robot");
                error = true;
            }
            if (elevateTurretTo(aimMan.getPitchToTarget()) == -1) {
                //the target is in the deadzone
                telemetry.addLine("target is in elevator deadzone, try moving the robot closer");
                error = true;
            }
            if (!error) {
                elevateTurretTo(aimMan.getPitchToTarget());

                //spin up flywheels and wait a bit to let everything move
                robot.flyWheel1.setPower(0.9);
                robot.flyWheel2.setPower(1.0);

                //open launcher
                robot.turretLauncher.setPosition(0);

                sleep(500);

                //launch ring, and go through reload sequence
                robot.turretLauncher.setPosition(0.75);
                sleep(250);

                //reset/prep for next shot
                elevateTurretTo(0);
                sleep(250);
                robot.flyWheel1.setPower(0);
                robot.flyWheel2.setPower(0);
            }*/
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
                boolean error = false;
                //move turret to aim at target
                if (rotateTurretTo(aimMan.getHeadingToTarget()) == -1) {
                    //the target is in deadzone
                    telemetry.addLine("target is in turret dead zone, try rotating the robot");
                    startOfCooldown = getRuntime();
                    error = true;
                }
                if (elevateTurretTo(aimMan.getPitchToTarget()) == -1) {
                    //the target is in the deadzone
                    telemetry.addLine("target is in elevator deadzone, try moving the robot closer");
                    startOfCooldown = getRuntime();
                    error = true;
                }
                if (!error) {
                    //spin up flywheels and wait a bit to let everything move
                    robot.flyWheel1.setPower(0.9);
                    robot.flyWheel2.setPower(1.0);

                    //open launcher
                    robot.turretLauncher.setPosition(0);

                    sleep(500);

                    //launch ring, and go through reload sequence
                    robot.turretLauncher.setPosition(0.75);
                    sleep(250);
                    //reset/prep for next shot
                    elevateTurretTo(0);
                    sleep(250);
                    robot.flyWheel1.setPower(0);
                    robot.flyWheel2.setPower(0);
                }

        }
    }
}

