package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareUltimateGoal(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_WITHOUT_ENCODERS if encoders are not installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.

        // Define and initialize ALL installed sensors.


    }
}
