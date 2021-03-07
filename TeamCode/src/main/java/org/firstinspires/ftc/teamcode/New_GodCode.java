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

@TeleOp(name="God", group="New")
///@Disabled
public class New_GodCode extends LinearOpMode {

    // Declare OpMode members.
    Map_Tank driveMap = new Map_Tank();
    Map_Lift liftMap = new Map_Lift();
    Map_Intake intakeMap = new Map_Intake();
    Map_Shooter shooterMap = new Map_Shooter();

    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        driveMap.init(hardwareMap);
        liftMap.init(hardwareMap);
        intakeMap.init(hardwareMap);
        shooterMap.init(hardwareMap);

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
            driveMap.frontLeftDrive.setPower(fld);
            driveMap.frontRightDrive.setPower(frd);
            driveMap.backLeftDrive.setPower(bld);
            driveMap.backRightDrive.setPower(brd);

            intakeMap.intake.setPower(0.0);
            shooterMap.shooter.setPower(0.0);
            liftMap.lift.setPower(0.0);


            //Shooter = bumper2 [32]
            double shootSpeed = 1.0;
            if (gamepad1.right_bumper) {
                shooterMap.shooter.setPower(shootSpeed);
            } else if (gamepad1.left_bumper) {
                shooterMap.shooter.setPower(-shootSpeed);
            } else {
                shooterMap.shooter.setPower(0.0);
            }


            //Lift [Trigger 2]
            double hopperSpeed = 1;
            if (gamepad1.dpad_up) {
                liftMap.lift.setPower(hopperSpeed);

            } else if (gamepad1.dpad_down) {
                liftMap.lift.setPower(-hopperSpeed);

            }else {
                liftMap.lift.setPower(0.0);
            }

            //Loader servo. Button [2]
            //Loader servo. Button [2]
            if (gamepad1.a) {
                shooterMap.loader.setPosition(shooterMap.stop);
            } else {
                shooterMap.loader.setPosition(shooterMap.start);
                telemetry.addData("Load", "A");

            }

            //Flicker servo. Button [2]
            if (gamepad1.x) {
                shooterMap.sideFlick.setPosition(shooterMap.start);
                telemetry.addData("Side", "X");

            } else {
                shooterMap.sideFlick.setPosition(shooterMap.stop);
            }

            //Flicker servo. Button [2]
            if (gamepad1.b) {
                shooterMap.frontFlick.setPosition(shooterMap.stop);

            } else {
                shooterMap.frontFlick.setPosition(shooterMap.start);
                telemetry.addData("Front", "B");

            }


           //Intake Controls= trigger [1]
           double intakeSpeed = 1.0;
           if (gamepad1.left_trigger >= 0.5) {
               intakeMap.intake.setPower(intakeSpeed);

           }else if
                (gamepad1.right_trigger >= 0.5){
                intakeMap.intake.setPower(-intakeSpeed);

            }else{
                intakeMap.intake.setPower(0.0);
            }


           /*
           //Strafing [1]
            double driveSpeed = .2;
            double  driveSpeed1 = 1;
            if (gamepad1.right_bumper) { //right
               driveMap.frontLeftDrive.setPower(-driveSpeed1);
               driveMap.frontRightDrive.setPower(driveSpeed1);
               driveMap.backLeftDrive.setPower(driveSpeed1);
               driveMap.backRightDrive.setPower(-driveSpeed1);
           }
           else if (gamepad1.left_bumper) { //left
               driveMap.frontLeftDrive.setPower(driveSpeed1);
               driveMap.frontRightDrive.setPower(-driveSpeed1);
               driveMap.backLeftDrive.setPower(-driveSpeed1);
               driveMap.backRightDrive.setPower(driveSpeed1);
           }
           else {
               driveMap.frontLeftDrive.setPower(fld);
               driveMap.frontRightDrive.setPower(frd);
               driveMap.backLeftDrive.setPower(bld);
               driveMap.backRightDrive.setPower(brd);
           }

            */

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

