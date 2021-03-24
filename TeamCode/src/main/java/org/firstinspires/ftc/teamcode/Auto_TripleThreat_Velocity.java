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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name= "I'm Feeling Stellar ~0~", group= "HII"
)
//@Disabled
public class Auto_TripleThreat_Velocity extends LinearOpMode {
    /* Declare OpMode members. */
    //HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private final ElapsedTime     runtime = new ElapsedTime();


    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    //private DcMotor shooter;
    private DcMotorEx shooter;
    private DcMotor lift;
    public Servo loader;

    static final double     FORWARD_SPEED = 0.3; //Drive speed
    //static final double     SHOOT_SPEED    = 1.0;
    double     SHOOT_VELOCITY    = 1300; //Max ticks per revoloution 103.6

    //static final double     PWRSHT_SPEED    = .9; //powershot
    static final double     stop   = 0.1; //loader servo positions
    static final double     mid    = -0.5; //loader servo positions
    static final double     start   = 1;



    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);

        frontLeftDrive  = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive   = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive  = hardwareMap.dcMotor.get("backRightDrive");
        //shooter         = hardwareMap.dcMotor.get("shooter");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        lift            = hardwareMap.dcMotor.get("hopper");

        loader  = hardwareMap.get(Servo.class, "loader");

        //Set motor direction
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);// kinda sus
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        lift.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // shooter.setMode(DcMotor.RunMode.VelocityControl);




        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
       // telemetry.addData("Path0",  "Starting at %7d :%7d",
              //  shooter.getVelocity());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        shooter.setVelocity(0);
        loader.setPosition(mid);

        //S1: Raise Lift to shoot height
        double Lift_SPEED = 0.6;
        runtime.reset();
        lift.setPower(Lift_SPEED);
        sleep(1000);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7))
        {
            telemetry.addData("Path", "Leg 1: %2.5f  Elapsed", runtime.seconds());
            telemetry.update();
        }

        //S2: drive to shoot point
        frontLeftDrive.setPower(-FORWARD_SPEED);
        frontRightDrive.setPower(-FORWARD_SPEED);
        backLeftDrive.setPower(-FORWARD_SPEED);
        backRightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) { //5
            telemetry.addData("Path", "Leg 1: %2.5f  Elapsed", runtime.seconds());
            telemetry.update();
        }


        //S3A: Lift stop. Shoot 1
        double v = 1150;
        sleep(500);
        lift.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        //Shoot
        sleep(500);
        shooter.setVelocity(v);
        //One
        for (int i = 0; i <= 2; i++) {
            sleep(100); //11000
            loader.setPosition(start);
            sleep(1000);
            loader.setPosition(stop);
            sleep(1000);
            loader.setPosition(start);
            sleep(1000);
            loader.setPosition(stop);
        }
/*
        //Two
        sleep(1000);
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);
        sleep(1000);
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);

        //Three
        sleep(1000);
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);
        sleep(1000);
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);

        //Fuck Me
        sleep(1000);
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);
        /*
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);
         */
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("Path", "Leg 1: %2.5f  Elapsed", runtime.seconds());
            telemetry.addData("Shooter: ", "  ", shooter.getVelocity());
            telemetry.update();
        }
        /*
        //S3B: Shoot again
        shooter.setPower(PWRSHT_SPEED);
        sleep(11000);
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4.0 )) {
            telemetry.addData("Path", "Leg 1: %2.5f  Elapsed", runtime.seconds());
            telemetry.update();
        }


        //S3C: Shoot again again
        shooter.setPower(PWRSHT_SPEED);
        sleep(11000);
        loader.setPosition(start);
        sleep(1000);
        loader.setPosition(stop);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 4.0 )) {
            telemetry.addData("Path", "Leg 1: %2.5f  Elapsed", runtime.seconds());
            telemetry.update();
        }

         */

        //S4: Forward to park line
        frontLeftDrive.setPower(-FORWARD_SPEED);
        frontRightDrive.setPower(-FORWARD_SPEED);
        backLeftDrive.setPower(-FORWARD_SPEED);
        backRightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0) ) { //2.4
            telemetry.addData("Path", "Leg 1: %2.5f  Elapsed", runtime.seconds());
            telemetry.update();
        }


        //S5: Stop on the line & stop running motors
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        shooter.setVelocity(0);
        loader.setPosition(stop);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
