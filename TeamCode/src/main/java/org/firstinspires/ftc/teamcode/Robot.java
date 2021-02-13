package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot {
    MecanumDrive driveTrain;

    public Robot(LinearOpMode opMode) {

        driveTrain= new MecanumDrive(   new MotorEx(opMode.hardwareMap, "frontLeft"),
                                        new MotorEx(opMode.hardwareMap, "frontRight"),
                                        new MotorEx(opMode.hardwareMap, "backLeft"),
                                        new MotorEx(opMode.hardwareMap, "backRight"));
    }
}
