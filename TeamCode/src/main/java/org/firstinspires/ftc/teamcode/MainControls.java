/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Main Control", group = "Concept")
//@Disabled
public class MainControls extends LinearOpMode {

    static final double INCREMENT   = 0.05;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    Robot robot = new Robot(this);
    DcMotor motorRF,motorRB,motorLF,motorLB;
    double  powery,powerx,powerw   = 0;
    boolean rampUp  = true;
    double input;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //motorRB = hardwareMap.get(DcMotor.class, "RB");
        //motorRF = hardwareMap.get(DcMotor.class, "RF");

        //motorLB = hardwareMap.get(DcMotor.class, "LB");
        //motorLF = hardwareMap.get(DcMotor.class, "LF");
        //motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorLB.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
            powery = gamepad1.left_stick_y;
            powerx = gamepad1.left_stick_x;
            powerw = gamepad1.right_stick_x;
            robot.driveTrain.driveRobotCentric(powerx, powery, powerw);

            telemetry.addData("1   Powery", "%5.2f", powery);
            telemetry.addData("2   Powerx", "%5.2f", powerx);
            telemetry.addData("3   Powerw", "%5.2f", powerw);


            telemetry.update();

//            motorRB.setPower(-powery + powerx - powerw);
            telemetry.addData("4   RightBack", "%5.2f", -powery + powerx - powerw);

  //          motorLF.setPower(-powery + powerx + powerw);
            telemetry.addData("5   LeftFront", "%5.2f", -powery + powerx + powerw);

    //        motorLB.setPower(-powery - powerx + powerw);
            telemetry.addData("6   LeftBack", "%5.2f", -powery - powerx + powerw);
      //      motorRF.setPower(-powery - powerx - powerw);
            telemetry.addData("7  RightFront", "%5.2f", -powery - powerx - powerw);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        motorRB.setPower(0);
        motorLB.setPower(0);
        motorLF.setPower(0);
        motorRF.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
