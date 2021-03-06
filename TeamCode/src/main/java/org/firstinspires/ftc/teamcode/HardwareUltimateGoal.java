package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/*
    NOTE -conventions for the measurement of distances and anlges-
    distances will be recorded and implemented in Standard units (meters), do not use the imperial system in your code
        this is because the metric system is easier to convert and work with than imperial units
    angles will be measured in radians
        this is due to how the Trig functions operate in java (they expect input, and return, values measured in radians)

    angle 0:
        for the robot, heading of 0 means that it's facing to the right (POV: looking toward the targets)
        for the turret, heading of 0 means facing toward the front of the robot

        for the horiz odometry, + change should mean turning CCW
                                - change should mean turning CW

        positive angles for the turret (relative to the robot) are turning away from the side with the control hubs
        positive angles, for the robot (relative to the field) are facing toward the goals


     front of the robot is the side the turret is facing in

     Coords format: x,y,z
     * Origin (0,0) : center of the 6x6(square) grid
     *      field x bounds [-1.79705m,+1.79705m]
     *      field y bounds [-1.79705m,+1.79705m]
     * z = height from the foam grid
     * x+ = toward red alliance station (right from audience area perspective)
     * x- = toward blue alliance station
     * y+ = toward the tower goal and power shot targets
     * y- = toward audience
 */


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
    public DcMotor  leftDrive;
    public DcMotor  rightDrive;
    public DcMotor  flyWheel1;
    public DcMotor  flyWheel2;
    public DcMotor  conveyor;
    public DcMotor intakePulley;
    public DcMotor turretRotator;
    public DcMotor turretElevator; //go bilda 53:1

    public DcMotor leftOdometry;
    public DcMotor rightOdometry;
    public DcMotor horizOdometry;

    public CRServo turretLauncher;

    public Servo wobbleGrabber;
    public Servo distanceServo;

    public DistanceSensor distanceSensor;

    //public TouchSensor touch1; //commented out bc it's not installed yet
    //public ColorSensor color1 = null; //commented out bc it's not installed yet


    //IMU Sensor
    public BNO055IMU imu;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    public double leftOffsetMultiplier;


    /* some variables for different measurements of the robot */ //TODO: keep up to date
    public final double turretHeight = 0.15; //5 + (13/16) inches, from the floor to the launch platform at rest, up to date but not 100% accurate
    public final double robotOdometryWidth = 0.12065;  // 4.75in, 12.065cm, up to date, but not 100% accurate // refers to the distance between the 2 side encoders
    public final double robotOdometryLength = 0.08255; // 3.25in, 8.255cm    //refers to the distance between the front odometry encoder and the imaginary line between the 2 side encoders
    public static final long LAUNCHER_TIME_TO_ROTATE = 1400; //out of date, needs testing, this number represents how long it takes for the continuous servo to rotate one full rotation at full power

    //directions of the odometry encoders, 1 == FORWARD; -1 == REVERSE
    public final short leftDirection = 1;
    public final short rightDirection = -1;
    public final short horizDirection = 1;
    //odometry allowed count offset (how close to the perfect position is allowed?)
    public final int sideOdoAllowedCountOffset = 50; //for the left and right odometry
    public final int horizOdoAllowedCountOffset = 50;

    //stats for the odometry encoders
    public final double ODOMETRY_COUNTS_PER_MOTOR_REV = 2048*4;
    public final double ODOMETRY_WHEEL_DIAMETER_METERS= 0.0508; //(2") For figuring circumference
    public final double ODOMETRY_COUNTS_PER_METER      = (ODOMETRY_COUNTS_PER_MOTOR_REV) / (ODOMETRY_WHEEL_DIAMETER_METERS * Math.PI);
    public final double ODOMETRY_METERS_PER_COUNT = 1.0 /ODOMETRY_COUNTS_PER_METER;


    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 32.0/24.0;  // This is < 1.0 if geared UP (to increase speed)
    public final double NADO_WHEEL_DIAMETER_METERS= 0.1016; //(4") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_COUNT = 1.0 / NADO_COUNTS_PER_METER;

    // stats for the Rev Core Hex motor
    public final double CORE_HEX_COUNTS_PER_MOTOR_REV = 288;  // 4 * 72(gear ratio)
    public final double CORE_HEX_DRIVE_GEAR_REDUCTION = 1;    // This is < 1.0 if geared UP
    public final double CORE_HEX_RADIANS_PER_COUNTS   = (2 * Math.PI) / (CORE_HEX_COUNTS_PER_MOTOR_REV * CORE_HEX_DRIVE_GEAR_REDUCTION); //  radians per rotation / effective counts per rotation

    // stats for the goBilda series 5201 53:1 motor
    public final double GO_BILDA_COUNTS_PER_MOTOR_REV   = 1497.325;
    public final double GO_BILDA_DRIVE_GEAR_REDUCTION   = 1; // This is < 1.0 if geared UP
    public final double GO_BILDA_RADIANS_PER_COUNTS     = (2 * Math.PI) / (GO_BILDA_COUNTS_PER_MOTOR_REV * GO_BILDA_DRIVE_GEAR_REDUCTION); //  radians per rotation / effective counts per rotation


    /* Constructor */
    public HardwareUltimateGoal(){

    }

    /*another constructor for testing, when there isn't an autonomous to write the heading and position files*/
    public HardwareUltimateGoal(String writesFilesAsRedTeam){
        writePositionHeading(new double[]{1.79705 - 0.8, -1.79705 + 0.225}, Math.PI/2); //TODO: update this as needed, use the default values in PositionAndTargetManager.java
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //read the offset
        //leftOffsetMultiplier= readLeftDriveEncoderOffsetMultiplier();

        // Define and Initialize Motors
        leftDrive   = hwMap.get(DcMotor.class, "leftDrive"); //main hub, motor port 0
        rightDrive  = hwMap.get(DcMotor.class, "rightDrive"); //main hub, motor port 1
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        flyWheel1   = hwMap.get(DcMotor.class, "flywheelLeft"); //second hub, motor port 0
        flyWheel2   = hwMap.get(DcMotor.class, "flywheelRight"); //second hub, motor port 1
        flyWheel1.setDirection(DcMotor.Direction.FORWARD);
        flyWheel2.setDirection(DcMotor.Direction.REVERSE);

        turretRotator = hwMap.get(DcMotor.class, "turretRotate"); //main hub, motor port 3
        turretElevator = hwMap.get(DcMotor.class, "turretElevator"); //main hub, motor port 2,
        conveyor       = hwMap.get(DcMotor.class, "conveyor"); //second hub, motor port 3
        intakePulley    = hwMap.get(DcMotor.class, "intakePulley"); //second hub, motor port 2,
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotator.setDirection(DcMotorSimple.Direction.REVERSE);

        // odometry: left and right odometry are plugged into the encoder slots for the drive motors on their corresponding sides
        // they parasite off of other motors encoder slots
        leftOdometry = hwMap.get(DcMotor.class, "flywheelRight");   //second hub, encoder port 1
        rightOdometry = hwMap.get(DcMotor.class, "flywheelLeft"); //second hub, encoder port 0
        horizOdometry = hwMap.get(DcMotor.class, "intakePulley");  //second hub, encoder port 2

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);
        turretElevator.setPower(0);
        turretRotator.setPower(0);
        conveyor.setPower(0);
        intakePulley.setPower(0);

        // Set run modes
        //encoders
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set to run with encoder
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //torqueNADO motor
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //torqueNADO motor
        turretRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //core hex motor //will run using a target position
        turretElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);// GoBilda 5201 series 53:1 //will run using a target position
        //set to run without encoder
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //flyWheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //commented ou for odometry
        //flyWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakePulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set zero behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakePulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        turretLauncher = hwMap.get(CRServo.class, "turretLaunchServo"); ///main hub servo port 1 // continuous rotation servo
        turretLauncher.setPower(0);
        // forward, here, means that the launch paddle moves in the direction that would launch the ring
        // stopped: 0
        // forward: [-1,0)
        // backward: (0,1]
        /*old, this is from when the servo was a normal servo*/
        //power 0 = full reverse; power 0.5 = stop; power 1 = full forward
        //position 0.5 = open for reload, position
        //-1 is forward (shoot)
        // .5 is ready to reload
        // .1 is ready to shoot
        //1 is back



        wobbleGrabber   = hwMap.get(Servo.class, "wobbleGrabber"); //main hub servo port 2 //360 servo
        wobbleGrabber.setPosition(0.8); //should be the open position, closed position is half a full rotation from open
        //position 0 = closed, position .8 = open?


        distanceServo = hwMap.get(Servo.class, "distanceServo"); //main hub servo port 3 //180 servo
        distanceServo.setPosition(0.5);
        //position 0.5: parallel to the front of the robot (folded in), position 1: pointed outwards, ready to scan
        //DO NOT SET BELOW 0.5 btw, or it will be pushing against the tower, which may cause some damage

        // Define and initialize ALL installed sensors.
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor"); //main hub I2C port 3
        //usage example: distanceSensor.getDistance(DistanceUnit.METER);
        /*
            the distances that should be detected for various ring numbers
            0 - ~10.8-11.5 CM
            1 - ~8-9cm
            4 - ~3.2-4 CM

            in code:

            if > 5 { if > 10 {0 rings} else {1 ring} } else {4 rings}
            or
            set as 0 rings
            if < 10 set as 1 ring
            if < 5 set as 4 rings
         */

        //touch1 = hwMap.touchSensor.get("touch_sensor");
        //color1 = hwMap.colorSensor.get("color1");


        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hwMap.get(BNO055IMU.class, "imu");
        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


    }

    ////////////////////////////// file for storing position and heading info after autonomous //////////////////////////////
    public static File positionXFile = AppUtil.getInstance().getSettingsFile("positionX.txt");
    public static File positionYFile = AppUtil.getInstance().getSettingsFile("positionY.txt");
    public static File headingFile = AppUtil.getInstance().getSettingsFile("heading.txt");
    public static File leftDriveEncoderOffsetMultiplierFile = AppUtil.getInstance().getSettingsFile("leftDriveEncoderOffsetMultiplier.txt");

    /**
     * method to write the position and heading of the robot to a file (overwriting any data already there)
     * in order to allow that information to persist after, for example, the execution of an autonomous opmode
     * @param position  the position of the robot, in meters, on a coordinate plane as described in comments at the beginning of PositionAndTargetManager.java
     * @param heading   the heading of the robot, in degrees
     */
    public static void writePositionHeading(double[] position, double heading) {
        ReadWriteFile.writeFile(positionXFile, String.valueOf(position[0]));
        ReadWriteFile.writeFile(positionYFile, String.valueOf(position[1]));

        ReadWriteFile.writeFile(headingFile, String.valueOf(heading));
    }

    /**
     * reads information from positionX.txt and positionY.txt to return the x,y position of the robot
     * @return a double array containing the [x,y] coordinates of the robot on the field, in meters
     */
    public static double[] readPosition() {
        return new double[]{Double.parseDouble(ReadWriteFile.readFile(positionXFile).trim()) , Double.parseDouble(ReadWriteFile.readFile(positionYFile).trim()) };
    }

    /**
     * reads information for heading.txt to return the heading of the robot
     * @return the heading of the robot, in degrees
     */
    public static double readHeading() {
        return Double.parseDouble(ReadWriteFile.readFile(headingFile).trim());
    }


    /**
     * writes the drive offset calculated in driveEncoderCalibration.java
     * @param offset
     */
    public static void writeLeftDriveEncoderOffsetMultiplier(double offset) {
        ReadWriteFile.writeFile(leftDriveEncoderOffsetMultiplierFile, String.valueOf(offset));
    }
    public static double readLeftDriveEncoderOffsetMultiplier() {
        return Double.parseDouble(ReadWriteFile.readFile(leftDriveEncoderOffsetMultiplierFile).trim());
    }


    ////////////////////////////// Movement and utility methods //////////////////////////////
    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    public double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * runs a given motor (that has an encoder) to a given position, at a given power
     * @param motor the motor to move
     * @param targetPosition    the position to move to
     * @param power the power to move at (must be positive)
     */
    public void runMotorToPosition(DcMotor motor, int targetPosition, double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(targetPosition);
        motor.setPower(0);
        if (targetPosition > motor.getCurrentPosition()) motor.setPower(power);
        else if (targetPosition < motor.getCurrentPosition()) motor.setPower(-power);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motor.isBusy()) {} //let the motor run to that position

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //TODO: maybe overload this method so that it's more usable for the autonomous

    // 2 methods that handle moving the robot for autonomous

    // get methods
    public double getRobotOdometryWidth() {
        return robotOdometryWidth;
    }
    public double getRobotOdometryLength() {
        return robotOdometryLength;
    }
    public short getLeftDirection() {
        return leftDirection;
    }
    public short getRightDirection() {
        return rightDirection;
    }
    public short getHorizDirection() {
        return horizDirection;
    }
    public int getSideOdoAllowedCountOffset() {
        return sideOdoAllowedCountOffset;
    }
    public int getHorizOdoAllowedCountOffset() {
        return horizOdoAllowedCountOffset;
    }
}
