package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;

@TeleOp (name="testing and hardware calibration", group = "UltimateGoal")
public class testingAndHardwareCalibration extends LinearOpMode {
    /*declare Opmode members, initialize some classes*/
    HardwareUltimateGoal robot          = new HardwareUltimateGoal();

    @Override
    public void runOpMode()
    {
        waitForStart();
        robot.init(hardwareMap);
        //firing sequence

        //prep shot
        elevateTurretTo(Math.toRadians(15)); //elevate the turret slightly to assist with the reload
        //wiggle the launching thing around a bit
        robot.turretLauncher.setPower(-0.3);
        sleep(300);
        robot.turretLauncher.setPower(0);
        sleep(100);
        robot.turretLauncher.setPower(-.5);
        sleep(300); //adjust timing
        robot.turretLauncher.setPower(0.15);
        elevateTurretTo(0);

        //aim
        rotateTurretTo(Math.PI / 6);
        telemetry.update();
        sleep(3000);
        elevateTurretTo(Math.toRadians(30));

        //fire
        //spin up flywheels and wait a bit to let everything move up to speed, the flywheels are not the same speed in order to create a spin
        robot.flyWheel1.setPower(.45);
        robot.flyWheel2.setPower(.5);
        sleep(500);
        //launch ring.
        // rotate the launch servo enough that the ring gets pushed into the flywheels, and the launcher is ready to accept the next ring
        robot.turretLauncher.setPower(-1);
        sleep(500); ///this number will change with testing


        //reset
        robot.flyWheel1.setPower(0);
        robot.flyWheel2.setPower(0);
        elevateTurretTo(0);
        rotateTurretTo(Math.PI);
        robot.turretLauncher.setPower(0.5);
    }

    /**
     * given the angle relative to the field, convert to the angle relative to the robot (front = 0), then move the turret to that angle
     * @param angle angle to target relative to field
     * @return if the target angle in in a dead zone, it returns -1, otherwise it returns 0 and rotates the turret to the needed position
     */
    public int rotateTurretTo(double angle) {
        double targetPosition; //angle to target, relative to field
        double robotHeading = 0; //double robotHeading = posTarMan.getRobotHeading();

        //heading relative to field -> heading relative to the robot
        targetPosition = angle - robotHeading;
        telemetry.addData("target angle", Math.toDegrees(targetPosition));
        //check if heading rel. to robot is in the deadzone, if so, return -1
        if (targetPosition > Math.toRadians(45) || targetPosition < -Math.toRadians(45)) return -1;
        //convert the heading rel. to robot into the needed encoder counts
        targetPosition /= robot.CORE_HEX_RADIANS_PER_COUNTS;
        telemetry.addData("target encoder position", targetPosition);
        //make sure that the robot rotates the best direction to reach goal

        //rotate to that position and return 0
        robot.runMotorToPosition(robot.turretRotator, (int) targetPosition, 0.1);
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

        robot.runMotorToPosition(robot.turretElevator, targetPos.intValue(), 0.1);
        return 0;
    }
    double elevationStep = 1;
    MathContext elevationMC = new MathContext( 16, RoundingMode.HALF_UP);
    BigDecimal elevationLastGuessDegrees = new BigDecimal("00.0000", elevationMC);
    double elevationGuessOffset = 1;
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
}
