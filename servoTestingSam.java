package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "servoTestingSam", group= "Linear OpMode")
public class servoTestingSam extends LinearOpMode {
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
    private Servo   stopper = null;

    double leftDrive;
    double rightDrive;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        wobble = hardwareMap.get(CRServo.class, "wobble");
        wobble2 = hardwareMap.get(CRServo.class, "wobble2");
        wobbleGrab = hardwareMap.get(Servo.class, "wobbleGrab");
        stopper = hardwareMap.get(Servo.class, "stopper");

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

        wobbleGrab.setPosition(0);
        stopper.setPosition(0);


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.x){
                wobbleGrab.setPosition(1);
            } else if (gamepad2.b){
                wobbleGrab.setPosition(0);
            }

            if (gamepad2.left_bumper) {
                wobble.setPower(1);
                wobble2.setPower(-1);
            } else if (gamepad2.right_bumper) {
                wobble.setPower(-1);
                wobble2.setPower(1);
            } else {
                wobble.setPower(0);
                wobble2.setPower(0);
            }

            //SIMPLE TANK//
            leftDrive = -gamepad1.left_stick_y;
            rightDrive = -gamepad1.right_stick_y;
            leftFront.setPower(leftDrive);
            leftBack.setPower(leftDrive);
            rightFront.setPower(rightDrive);
            rightBack.setPower(rightDrive);
            /////////////

            //PRECISION DRIVE//
            if (gamepad1.dpad_up){
                leftFront.setPower(1);
                leftBack.setPower(1);
                rightFront.setPower(1);
                rightBack.setPower(1);
            }
            if (gamepad1.dpad_down){
                leftFront.setPower(-1);
                leftBack.setPower(-1);
                rightFront.setPower(-1);
                rightBack.setPower(-1);
            }
            ////////////////////


            //STRAFING LEFT//
            if (gamepad1.dpad_left) {
                leftFront.setPower(-1);
                leftBack.setPower(1);
                rightFront.setPower(1);
                rightBack.setPower(-1);
            }
            /////////////////

            //STRAFING RIGHT//
            if (gamepad1.dpad_right) {
                leftFront.setPower(1);
                leftBack.setPower(-1);
                rightFront.setPower(-1);
                rightBack.setPower(1);
            }
            //////////////////


            //SHOOTER//
            if (gamepad2.a) {
                leftShooter.setPower(0.8);
                rightShooter.setPower(-0.8);
            } else if (gamepad2.y) {
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }
            //////////


            if (gamepad2.dpad_up) {
                conveyor.setPower(-1);
                intake.setPower(1);
            } else if (gamepad2.dpad_down) {
                conveyor.setPower(1);
                intake.setPower(-1);
            } else
                conveyor.setPower(0);
            intake.setPower(0);

        }

        if (gamepad1.y){
            stopper.setPosition(1);
        } else if (gamepad1.a){
            stopper.setPosition(0);
        }
    }

}


