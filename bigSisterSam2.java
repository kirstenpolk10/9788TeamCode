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


public class bigSisterSam2 extends LinearOpMode{

    private DcMotor leftFront    = null;
    private DcMotor rightFront   = null;
    private DcMotor leftBack     = null;
    private DcMotor rightBack    = null;

    private DcMotor intake       = null;
    private DcMotor conveyor     = null;

    private DcMotor leftShooter  = null;
    private DcMotor rightShooter = null;

    //private Servo ;

    double leftSidePower;
    double rightSidePower;
    //private Sero;
    //private Servo;



    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "front_left");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront = hardwareMap.get(DcMotor.class, "front_right");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "back_left");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack = hardwareMap.get(DcMotor.class, "back_right");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


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

        /*Pushy = hardwareMap.servo.get("Pushy");
        Pushy.setPosition(0);*/

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
            leftFront.setPower(leftSidePower);
            leftBack.setPower(leftSidePower);
            rightSidePower = (-gamepad1.right_stick_y);
            rightFront.setPower(rightSidePower);
            rightBack.setPower(rightSidePower);



            if (gamepad2.dpad_up) {
                conveyor.setPower(1);
                intake.setPower(0.75);
            } else {
                conveyor.setPower(0);
                if (gamepad2.dpad_down) {
                    intake.setPower(-0.75);
                } else {
                    intake.setPower(0);
                }
            }


            /*if(gamepad1.x){
                shovelL.setPosition(0.2);
                shovelR.setPosition(1);
            }else if(gamepad1.b) {
                shovelL.setPosition(1);
                shovelR.setPosition(0.2); */

            //}

            if (gamepad2.a) {
                leftShooter.setPower(0.5);
                rightShooter.setPower(-0.5);
            } else if (gamepad2.y) {
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }

            /*if(gamepad1.x){
                Pushy.setPosition(0);
            }else if(gamepad1.y) {
                Pushy.setPosition(1);
            }*/






        }
    }
}









