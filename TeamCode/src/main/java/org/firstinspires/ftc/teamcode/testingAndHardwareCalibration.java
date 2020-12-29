package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="testing and hardware calibration", group = "UltimateGoal")
public class testingAndHardwareCalibration extends LinearOpMode {
    /*declare Opmode members, initialize some classes*/
    HardwareUltimateGoal robot          = new HardwareUltimateGoal();

    @Override
    public void runOpMode()
    {
        waitForStart();
        robot.init(hardwareMap);
        sleep(500);


        while(opModeIsActive()) {
            robot.flyWheel1.setPower(.9);
            robot.flyWheel2.setPower(.8);
        }/*
        robot.turretLauncher.setPower(-.4);

        sleep(2000);

        robot.turretLauncher.setPower(0.15);

        sleep(2000);

        robot.turretLauncher.setPower(-1);

        robot.wobbleGrabber.setPosition(1);
        sleep(2000);

        robot.flyWheel1.setPower(0);
        robot.flyWheel2.setPower(0); */
    }
}
