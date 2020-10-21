package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for the robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *  more motors, sensors, servos, etc will be added as needed
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 *
 *
 * the purpose of this class is to act as a centralized method for initializing the hardware on the
 * robot, so that the addition of hardware elements can be accounted for in one central area, reducing
 * the risk of error in the event the robot is updated
 */
public class HardwareUltimateGoal {
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  flyWheel = null;
    public DcMotor  conveyor1 = null;

    public DcMotor turretElevator = null;
    public Servo turretRotator = null;

    public TouchSensor touch1 = null;
    public ColorSensor color1 = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* some variables for different measurements of the robot */ //TODO: keep up to date
    public double turretHeight = 0;
    public double driveWheelRadius = 0.0508; //2" measured in meters
    public double robotWidth = 0.4572; //18" measured in meters

    /* Constructor */
    public HardwareUltimateGoal(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive   = hwMap.get(DcMotor.class, "left_drive");
        rightDrive  = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        flyWheel        = hwMap.get(DcMotor.class, "flywheel");
        conveyor1       = hwMap.get(DcMotor.class, "conveyor1");
        turretElevator  = hwMap.get(DcMotor.class, "elevator");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        flyWheel.setPower(0);
        conveyor1.setPower(0);

        // May want to use RUN_WITHOUT_ENCODERS if encoders are not installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        turretRotator = hwMap.get(Servo.class, "turretRotator");
        turretRotator.setPosition(0);

        // Define and initialize ALL installed sensors.
        touch1 = hwMap.touchSensor.get("touch_sensor");
        color1 = hwMap.colorSensor.get("color1");
    }


    //methods for the turret that set things to precise positions
}
