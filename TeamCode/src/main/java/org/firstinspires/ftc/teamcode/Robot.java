package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot {
    MecanumDrive driveTrain;


    //motorRB = hardwareMap.get(DcMotor.class, "RB");
    //motorRF = hardwareMap.get(DcMotor.class, "RF");

    //motorLB = hardwareMap.get(DcMotor.class, "LB");
    //motorLF = hardwareMap.get(DcMotor.class, "LF");

    public Robot(LinearOpMode opMode) {
        driveTrain = new MecanumDrive(  new MotorEx(opMode.hardwareMap, "LF"),
                                        new MotorEx(opMode.hardwareMap, "RF"),
                                        new MotorEx(opMode.hardwareMap, "LB"),
                                        new MotorEx(opMode.hardwareMap, "RB"));

        //MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight,
        //                                        backLeft, backRight);
    }
}
