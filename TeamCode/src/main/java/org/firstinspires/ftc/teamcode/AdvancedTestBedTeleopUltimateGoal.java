package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//TODO: 10/21/2020 eventually, change this from a testbed type thing (with all the telemetry), to a final product
@TeleOp(name="TeleOP Test Bed Ultimate Goal", group="UltimateGoal")
public class AdvancedTestBedTeleopUltimateGoal extends LinearOpMode {
    /* Declare OpMode members. initialize some classes */
    HardwareUltimateGoal robot          = new HardwareUltimateGoal();
    PositionAndTargetManager posTarMan  = new PositionAndTargetManager(robot, true); //TODO: 10/21/2020 change true to false if on team blue
    ElapsedTime runtime                 = new ElapsedTime();
    AimAssist aimMan                    = new AimAssist(robot, posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.bestTargetPosition(0));

    double currentTurretHeading = 0;
    double currentTurretPitch   = 0;

    @Override
    public void runOpMode()
    {
        // declare some variables if needed
        double totalLeftCounts  = 0;
        double totalRightCounts = 0;

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
            totalLeftCounts     = robot.leftDrive.getCurrentPosition();
            totalRightCounts    = robot.rightDrive.getCurrentPosition();
            tankControls(gamepad1.right_stick_y, gamepad1.left_stick_y);

            if (gamepad1.a) { // if driver presses A, change targets
                posTarMan.bestTargetPosition(runtime.seconds());
            }


            // update position and aim managers
            double leftRevs     = (totalLeftCounts - robot.leftDrive.getCurrentPosition()) / robot.NADO_COUNTS_PER_MOTOR_REV; //left rotations since last count
            double rightRevs    = (totalRightCounts - robot.rightDrive.getCurrentPosition()) / robot.NADO_COUNTS_PER_MOTOR_REV; //right rotations since last count
            posTarMan.update(leftRevs, rightRevs);

            aimMan.update(posTarMan.getRobotPosition(), posTarMan.getRobotHeading(), posTarMan.getTargetPosition());

            //telemetry
            telemetry.addLine("position information");
            telemetry.addData("x", posTarMan.getRobotPosition()[0]);
            telemetry.addData("y", posTarMan.getRobotPosition()[1]);
            telemetry.addData("heading", posTarMan.getRobotHeading());

            telemetry.addLine("turret information");
            telemetry.addData("heading", currentTurretHeading);
            telemetry.addData("pitch", currentTurretPitch);
            telemetry.addData("heading to target", aimMan.getHeadingToTarget());
            telemetry.addData("pitch to target", aimMan.getPitchToTarget());

            // automated movement (turret)
            rotateTurretTo(aimMan.headingToTarget);

        }
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

    public void rotateTurretTo(double degrees) {
        //TODO: 10/21/2020 update this to account for gear reduction and stuff, also, may need to change some things to adjust the target heading for robot heading
        double pos = robot.turretRotator.getPosition();
        //correct for angles too big or too small
        while(degrees > 180) degrees    -= 360;
        while(degrees < -180) degrees   += 360;

        if (degrees > currentTurretHeading) {
            currentTurretHeading    = ((robot.turretRotator.getPosition() - 0.5) * 360)/*turret heading*/ - posTarMan.robotHeading;
            pos                     = ((degrees - currentTurretHeading) / 180) + 0.5;

        } else if (degrees < currentTurretHeading) {
            currentTurretHeading    = ((robot.turretRotator.getPosition() - 0.5) * 360)/*turret heading*/ - posTarMan.robotHeading;
            pos                     = ((degrees + currentTurretHeading) / 180) + 0.5;
        }
        robot.turretRotator.setPosition(pos);
    }
    public void elevateTurretTo(double degrees) {
        //TODO: 10/21/2020 finish this, right now it's using a pulley or something
        double pos;
        //correct for angles too big or too small
        while(degrees > 180) degrees -= 180;
        while(degrees < 0) degrees += 180;
        //convert rotations to ticks of the encoder
        //adjust for
    }
}

