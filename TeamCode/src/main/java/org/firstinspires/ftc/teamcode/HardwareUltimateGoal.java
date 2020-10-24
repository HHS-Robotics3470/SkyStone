package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

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
    //public DcMotor  flyWheel = null; //commented out bc it's not installed yet
    //public DcMotor  conveyor1 = null; //commented out bc it's not installed yet

    public DcMotor turretElevator = null;
    public Servo turretRotator = null;

    //public TouchSensor touch1 = null; //commented out bc it's not installed yet
    //public ColorSensor color1 = null; //commented out bc it's not installed yet


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* some variables for different measurements of the robot */ //TODO: keep up to date
    public double turretHeight = 0;
    public double robotWidth = 0.4572; //18" measured in meters
    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 1.0;    // This is < 1.0 if geared UP
    public final double NADO_WHEEL_DIAMETER_METERS= 0.1016; //(4") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_REV = NADO_COUNTS_PER_METER / (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION);
    // stats for the NeveRest motor
    public final double NEVE_COUNTS_PER_MOTOR_REV = 448; // 28 * 16(gear ratio)
    public final double NEVE_DRIVE_GEAR_REDUCTION = 1.0;    // This is < 1.0 if geared UP
    public final double NEVE_WHEEL_DIAMETER_METERS= 0.1016; //(4") For figuring circumference
    public final double NEVE_COUNTS_PER_METER      = (NEVE_COUNTS_PER_MOTOR_REV * NEVE_DRIVE_GEAR_REDUCTION) /
            (NEVE_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NEVE_METERS_PER_REV = NEVE_COUNTS_PER_METER / (NEVE_COUNTS_PER_MOTOR_REV * NEVE_DRIVE_GEAR_REDUCTION);

    /* Constructor */
    public HardwareUltimateGoal(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive   = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hwMap.get(DcMotor.class, "rightDrive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        //flyWheel        = hwMap.get(DcMotor.class, "flywheel");
        //conveyor1       = hwMap.get(DcMotor.class, "conveyor1");
        turretElevator  = hwMap.get(DcMotor.class, "turretElevator");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //flyWheel.setPower(0);
        //conveyor1.setPower(0);

        // May want to use RUN_WITHOUT_ENCODERS if encoders are not installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        turretRotator = hwMap.get(Servo.class, "turretServo");
        turretRotator.setPosition(0);

        // Define and initialize ALL installed sensors.
        //touch1 = hwMap.touchSensor.get("touch_sensor");
        //color1 = hwMap.colorSensor.get("color1");
    }

    ////////////////////////////// file for storing position and heading info after autonomous //////////////////////////////
    public static File positionXFile = AppUtil.getInstance().getSettingsFile("positionX.txt");
    public static File positionYFile = AppUtil.getInstance().getSettingsFile("positionY.txt");
    public static File headingFile = AppUtil.getInstance().getSettingsFile("heading.txt");


    public static void writePositionHeading(double[] position, double heading) {
        ReadWriteFile.writeFile(positionXFile, String.valueOf(position[0]));
        ReadWriteFile.writeFile(positionYFile, String.valueOf(position[1]));

        ReadWriteFile.writeFile(headingFile, String.valueOf(heading));
    }

    public static double[] readPosition() {
        return new double[]{Double.parseDouble(ReadWriteFile.readFile(positionXFile).trim()) , Double.parseDouble(ReadWriteFile.readFile(positionYFile).trim()) };
    }

    public static double readHeading() {
        return Double.parseDouble(ReadWriteFile.readFile(headingFile).trim());
    }

    ////////////////////////////// Movement methods //////////////////////////////



}
