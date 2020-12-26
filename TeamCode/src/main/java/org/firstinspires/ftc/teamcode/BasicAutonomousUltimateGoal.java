package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        /*
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
