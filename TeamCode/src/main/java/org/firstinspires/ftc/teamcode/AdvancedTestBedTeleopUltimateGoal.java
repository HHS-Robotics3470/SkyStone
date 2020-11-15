package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        currentTurretHeading = robot.turretRotator.getPosition() * 2 * Math.PI; //turret heading in radians relative to the robot
        currentTurretHeading = posTarMan.getRobotHeading() - ( currentTurretHeading - (Math.PI/4) ); //turret heading relative to field
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

            if (gamepad1.a && !abort) { // if driver presses A, change targets
                posTarMan.bestTargetPosition(runtime.seconds());
            }

            if (gamepad1.b) {
                telemetry.addLine("motor encoder counts");
                telemetry.addData("left motor", robot.leftDrive.getCurrentPosition());
                telemetry.addData("right motor", robot.rightDrive.getCurrentPosition());
                telemetry.update();
                sleep(2000);
            }


            //abort button
            if (gamepad1.x) {
                abort = !abort;
            }


          // update position and aim managers
            leftCounts     = robot.leftDrive.getCurrentPosition(); //total left encoder counts
            rightCounts    = robot.rightDrive.getCurrentPosition();//total right encoder counts
            posTarMan.update(leftCounts, rightCounts);

            aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());

            //telemetry
            telemetry.addLine("position information");
            telemetry.addData("x", posTarMan.getRobotPosition()[0]);
            telemetry.addData("y", posTarMan.getRobotPosition()[1]);
            telemetry.addData("heading", Math.toDegrees(posTarMan.getRobotHeading()));

            telemetry.addLine("turret information");
            telemetry.addData("heading", currentTurretHeading);
            telemetry.addData("pitch", currentTurretPitch);
            telemetry.addData("heading to target", aimMan.getHeadingToTarget());
            telemetry.addData("pitch to target", aimMan.getPitchToTarget());

            telemetry.addLine("target info");
            telemetry.addData("target", posTarMan.getCurrentTarget());
            telemetry.addData("x", posTarMan.getTargetPosition()[0]);
            telemetry.addData("y", posTarMan.getTargetPosition()[1]);
            telemetry.addData("z", posTarMan.getTargetPosition()[2]);

            telemetry.update();

          // automated movement (turret)
            if (!abort) {
                //rotateTurretTo(aimMan.headingToTarget);
            }
        }

        //after opMode, save current position and heading for reasons
        //HardwareUltimateGoal.writePositionHeading(posTarMan.getRobotPosition(), posTarMan.getRobotHeading());
    }

    public void basicStickControls(double x, double y) {
        double leftPower    = -y;
        double rightPower   = -y;
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
     */
    public void rotateTurretTo(double angle) {
        //TODO: 10/21/2020 update this to account for gear reduction and stuff, also, may need to change some things to adjust the target heading for robot heading
        double pos = robot.turretRotator.getPosition();
        // correct for angles too big or too small
        //while(angle > Math.PI) angle    -= Math.PI;
        //while(angle < -Math.PI) angle   += Math.PI;
        currentTurretHeading = robot.turretRotator.getPosition() * 2 * Math.PI; //turret heading in radians relative to the robot
        currentTurretHeading = posTarMan.getRobotHeading() - ( currentTurretHeading - (Math.PI/4) ); //turret heading relative to field

        if (angle > currentTurretHeading) {
            pos                     = ((angle - currentTurretHeading) / (2*Math.PI));

        } else if (angle < currentTurretHeading) {
            pos                     = ((angle + currentTurretHeading) / (2*Math.PI));
        }
        robot.turretRotator.setPosition(pos);
    }
    public void elevateTurretTo(double angle) {
        //TODO: 10/21/2020 finish this, right now it's using a pulley or something
        double pos;
        //correct for angles too big or too small
        while(angle > Math.PI/2) angle -= Math.PI/2;
        while(angle < 0) angle += Math.PI/2;
        //convert rotations to ticks of the encoder

        // first, adjust the degree relative to robot, to degree relative to turret

        // x = encoder counts of motor, y = degrees of the elevator
        // x/1440 * robot.ELEVATORGEARREDUCTION = y

        //motor position [0,1], encoder counts [0,1440], degrees [0,45)



        //adjust for
    }
}

