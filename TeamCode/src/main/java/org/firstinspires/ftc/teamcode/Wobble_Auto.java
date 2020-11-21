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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Wobble Auto", group = "Test")
//@Disabled
public class Wobble_Auto extends LinearOpMode {
    private Map_Auto_Tank robot        = new Map_Auto_Tank(); // use the class created to define a Pushbot's hardware
    private final ElapsedTime     runtime = new ElapsedTime();

    /*
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)robot.sensorRange;


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);

    private final DcMotor frontLeft = robot.frontLeftDrive;
    private final DcMotor frontRight = robot.frontRightDrive;
    private final DcMotor backLeft = robot.backLeftDrive;
    private final DcMotor backRight = robot.backRightDrive;
    private final DcMotor shoot = robot.shooter;
    private final DcMotor hopper = robot.hopper;

    static final double     DRIVE_SPEED = 0.6;
    static final double     STRAFE_SPEED  =  0.5;
    static final double     SHOOT_SPEED =  0.5;
    static final double     HOPPER_SPEED  =  0.5;

    //Mounting height of 2m sensor in MM
    static final double     snsrMount  =  101.6; //
    double stackHeight = robot.sensorRange.getDistance(DistanceUnit.MM) - snsrMount;


     */
    private final DcMotor frontLeft = robot.frontLeftDrive;
    private final DcMotor frontRight = robot.frontRightDrive;
    private final DcMotor backLeft = robot.backLeftDrive;
    private final DcMotor backRight = robot.backRightDrive;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.init(hardwareMap);



        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        encoderDrive(robot.DRIVE_SPEED,  30,  30, 10.0);  // S1: Forward to Start Stack
        sleep(1000);     // pause for servos to move
        //getStackHeight idk //S2: Scan Stack to determine which target to go for
        sleep(1000);

        if (robot.stackHeight <= 500 || robot.stackHeight >= 350) //1 ring = .75; Tar B [350-500mm]
        {
            encoderDrive(robot.DRIVE_SPEED, 96, 96, 10.0); // S3: Forward 96in
            sleep(1000);
            //S4: Drop Woddle
            sleep(1000);
            encoderDrive(robot.DRIVE_SPEED, -24, -24, 10.0); // S5: Reverse 24in. Park
            sleep(1000);
        }
        else if (robot.stackHeight >= 800 ) //4 ring = 3; Tar C [800-1100mm]
        {
            encoderDrive(robot.DRIVE_SPEED, 30, 30, 10.0); // S3: Strafe right 24in idk
            sleep(1000);
            encoderDrive(robot.DRIVE_SPEED, 96, 96, 10.0); // S4: Forward 96in
            sleep(1000);
            // S5: Drop woddle
            sleep(1000);
            encoderDrive(robot.DRIVE_SPEED, -60, -60, 10.0); // S6: Backward 60. Park
            sleep(1000);


        }
        else //0 ring = 0; Tar A [0-200 mm]
        {

            encoderDrive(robot.DRIVE_SPEED, 30, 30, 10.0); // S3: Strafe right 24in idk
            sleep(1000);
            encoderDrive(robot.DRIVE_SPEED, 30, 30, 10.0); // S4: Forward 30in. Park
            sleep(1000);
            // S5: Drop woddle

        }



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }




    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
/*
    public void positonA() {
        stackHeight = this.stackHeight;
        encoderDrive(DRIVE_SPEED, 30, 30, 10.0); // S3: Strafe right 24in idk
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 30, 30, 10.0); // S4: Forward 30in. Park
        sleep(1000);
        // S5: Drop woddle

    }

 */

    public void encoderDrive(double speed, double leftInches,
                             double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
            newRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));

            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}