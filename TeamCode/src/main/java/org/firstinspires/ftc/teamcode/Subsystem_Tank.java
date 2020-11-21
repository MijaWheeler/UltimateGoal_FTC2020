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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tank Drive ONLY", group="Subsystem")
//@Disabled
public class Subsystem_Tank extends LinearOpMode {

    // Declare OpMode members.
    //HardwareMap robot       = new HardwareMap(); // use the class created to define a Pushbot's hardware
    Map_Tank robot = new Map_Tank();
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double magnitudeLeft = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double magnitudeRight = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double robotAngleLeft = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double robotAngleRight = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
            final double fld = magnitudeLeft * Math.sin(robotAngleLeft);
            final double frd = magnitudeRight * Math.cos(robotAngleRight);
            final double bld = magnitudeLeft * Math.cos(robotAngleLeft);
            final double brd = magnitudeRight * Math.sin(robotAngleRight);

            //Initial Motor Speeds
            robot.frontLeftDrive.setPower(fld);
            robot.frontRightDrive.setPower(frd);
            robot.backLeftDrive.setPower(bld);
            robot.backRightDrive.setPower(brd);

            //Strafing
            double strafeSpeed = 1;
            if (gamepad1.right_bumper) { //right
                robot.frontLeftDrive.setPower(-strafeSpeed);
                robot.frontRightDrive.setPower(strafeSpeed);
                robot.backLeftDrive.setPower(strafeSpeed);
                robot.backRightDrive.setPower(-strafeSpeed);
            }
            else if (gamepad1.left_bumper) { //left
                robot.frontLeftDrive.setPower(strafeSpeed);
                robot.frontRightDrive.setPower(-strafeSpeed);
                robot.backLeftDrive.setPower(-strafeSpeed);
                robot.backRightDrive.setPower(strafeSpeed);
            }
            else {
                robot.frontLeftDrive.setPower(fld);
                robot.frontRightDrive.setPower(frd);
                robot.backLeftDrive.setPower(bld);
                robot.backRightDrive.setPower(brd);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

