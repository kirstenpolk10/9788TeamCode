package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@TeleOp(name = "bigSisterSam2", group= "OpMode")
//@Disabled


public class bigSisterSam3 extends LinearOpMode {

    private DcMotor left = null;
    private DcMotor right = null;

    private DcMotor intake = null;
    private DcMotor conveyor = null;

    private DcMotor leftShooter = null;
    private DcMotor rightShooter = null;

    private Servo Pushy;

    double leftSidePower;
    double rightSidePower;
    //private Servo shovelR        = null;
    //private Servo shovelL        = null;


    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left = hardwareMap.get(DcMotor.class, "left");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setDirection(DcMotor.Direction.FORWARD);
        right = hardwareMap.get(DcMotor.class, "right");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter = hardwareMap.get(DcMotor.class, "launcher");
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setDirection(DcMotor.Direction.FORWARD);
        rightShooter = hardwareMap.get(DcMotor.class, "launcher2");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);


        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        Pushy = hardwareMap.servo.get("Pushy");
        Pushy.setPosition(0.1);

       /* shovelL.setPosition(1);
        shovelL = hardwareMap.get(Servo.class, "leftShovel");
        shovelR.setPosition(0.2);
        shovelR = hardwareMap.get(Servo.class, "rightShovel");*/

        waitForStart();
        while (opModeIsActive()) {

           /* leftBack.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
            leftFront.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
            rightFront.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
            rightBack.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);*/
            leftSidePower = (-gamepad1.left_stick_y);
            left.setPower(leftSidePower);
            rightSidePower = (-gamepad1.right_stick_y);
            right.setPower(rightSidePower);


            if (gamepad2.dpad_up) {
                conveyor.setPower(1);
                intake.setPower(0.75);
            } else if (gamepad2.dpad_down) {
                conveyor.setPower(-1);
                intake.setPower(-0.75);
            } else {
                intake.setPower(0);
                conveyor.setPower(0);
            }
        }



            /*if(gamepad1.x){
                shovelL.setPosition(0.2);
                shovelR.setPosition(1);
            }else if(gamepad1.b) {
                shovelL.setPosition(1);
                shovelR.setPosition(0.2); */

        //}

        if (gamepad2.dpad_left) {
            leftShooter.setPower(0.5);
            rightShooter.setPower(-0.5);
        } else if (gamepad2.dpad_right) {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        if (gamepad1.dpad_down) {
            Pushy.setPosition(0.1);
        } else if (gamepad2.dpad_up) {
            Pushy.setPosition(1);
        }


    }
}











