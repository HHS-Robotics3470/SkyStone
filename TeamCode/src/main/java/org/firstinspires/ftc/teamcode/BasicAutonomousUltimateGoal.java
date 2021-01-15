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
        HardwareUltimateGoal.writePositionHeading(double[] position; double heading;)

        //---------------------------------------------------------------------------------------\\

        // Step through each leg of the path,

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
}
