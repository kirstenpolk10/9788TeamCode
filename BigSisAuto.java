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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


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

@Autonomous(name="Pushbot: BigSisAuto", group="Pushbot")

public class BigSisAuto extends LinearOpMode {


    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime tfTime = new ElapsedTime();
    private static double tfSenseTime = 1;


    static final double     FORWARD_SPEED = 0.6;
    static final double     BACKWARD_SPEED = -0.6;
    static final double     TURN_SPEED    = 0.5;
    public ElapsedTime drivetime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public String StackSize = "None";



    private static final String VUFORIA_KEY =
            "AdzQ2ID/////AAABmThO+al+t0bqpiRTJ7x7MCQa37ZKlqlToYy/JxYbErfT1+jNdP8BvYT/juE2rfYLabYOaNlzqQ7UlLE547Rv+5aUeLWwuEoLOpa+7XradL3bwHmEmysPH7hD8jYnuZqFdVKvw/IuRkfQ664KpZPwLE3coupFkk3O0JANWUpeIBK4zssHrxDDhxJTpE3Fz1rTjxIWRO26tjTuYhHXN6affzAakoe6ZxhhfqrUFFJLYIUFWVQE6ABb2OCJ1UNb6txTXU15v2sjh936RZQDlqMce8rMUpLOFOjQt6K0nvYbHmY/u8yWSqoYdFNXJ5s2bkZvxXEJdGS6cf8JOHV/k+XrHGc7CVpXeCPQsDb8+C02U7wK";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public enum WobbleTargetZone {
        BLUE_A,
        BLUE_B,
        BLUE_C,

    }

    WobbleTargetZone Square = WobbleTargetZone.BLUE_A;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftShooter = null;
    private DcMotor rightShooter = null;
    private DcMotor conveyor = null;
    private DcMotor intake = null;
    private CRServo wobble = null;
    private CRServo wobble2 = null;
    private Servo   wobbleGrab = null;

    @Override
    public void runOpMode() {


        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setDirection(DcMotor.Direction.FORWARD);

        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);

        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        wobble = hardwareMap.get(CRServo.class, "wobble");
        wobble2 = hardwareMap.get(CRServo.class, "wobble2");
        wobbleGrab = hardwareMap.get(Servo.class, "wobbleGrab");




        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        conveyor.setPower(0);
        intake.setPower(0);
        wobble.setPower(0);
        wobble2.setPower(0);
        wobbleGrab.setPosition(0);



        double leftDrive;
        double rightDrive;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        tfTime.reset();
        if (opModeIsActive()) {
            while (tfTime.time() < tfSenseTime && opModeIsActive()) {
                if (tfod !=null) {

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

// step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            StackSize = recognition.getLabel();
                            //telemetry.addData("Target", Target);
                            if (StackSize == "Quad") {
                                Square = WobbleTargetZone.BLUE_C;
                                telemetry.addData("Square", Square);
                            } else if (StackSize == "Single") {
                                Square = WobbleTargetZone.BLUE_B;
                                telemetry.addData("Square", Square);

                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
        drivetime.reset();
        if (tfod != null) {
            tfod.shutdown();
        }




        drivetime.reset();
        switch(Square) {
            case BLUE_A:
                telemetry.addData("Going to RED A", "Target Zone");
                wobbleGrab.setPosition(0);
                leftBack.setPower(FORWARD_SPEED);
                leftFront.setPower(FORWARD_SPEED);
                rightBack.setPower(FORWARD_SPEED);
                rightFront.setPower(FORWARD_SPEED);
                sleep(5000);



                drivetime.reset();
                sleep(1000);
                break;
            case BLUE_B:
                telemetry.addData("Going to RED B", "Target Zone");
                wobbleGrab.setPosition(0);
                leftBack.setPower(FORWARD_SPEED);
                leftFront.setPower(FORWARD_SPEED);
                rightBack.setPower(FORWARD_SPEED);
                rightFront.setPower(FORWARD_SPEED);
                sleep(8000);
                leftBack.setPower(BACKWARD_SPEED);
                leftFront.setPower(BACKWARD_SPEED);
                rightBack.setPower(BACKWARD_SPEED);
                rightFront.setPower(BACKWARD_SPEED);
                sleep(1000);


                drivetime.reset();
                sleep(1000);

                break;
            case BLUE_C:
                telemetry.addData("Going to Red A", "Target Zone");
                wobbleGrab.setPosition(0);
                leftBack.setPower(FORWARD_SPEED);
                leftFront.setPower(FORWARD_SPEED);
                rightBack.setPower(FORWARD_SPEED);
                rightFront.setPower(FORWARD_SPEED);
                sleep(10000);
                leftBack.setPower(BACKWARD_SPEED);
                leftFront.setPower(BACKWARD_SPEED);
                rightBack.setPower(BACKWARD_SPEED);
                rightFront.setPower(BACKWARD_SPEED);
                sleep(3000);
                drivetime.reset();
                break;

        }
        /*back_left.setPower(FORWARD_SPEED);
        back_right.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update(); *
        }

        // Step 1:  Drive forward for 3 seconds

        /*robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        robot.leftDrive.setPower(TURN_SPEED);
        robot.rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        robot.leftDrive.setPower(-FORWARD_SPEED);
        robot.rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftClaw.setPosition(1.0);
        robot.rightClaw.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); */
    }














    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters= new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);



    }

}