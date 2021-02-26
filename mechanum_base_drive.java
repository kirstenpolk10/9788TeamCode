package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="mechanum_base_drive", group="Linear Opmode")
public class mechanum_base_drive extends LinearOpMode {

    private DcMotor front_right = null;
    private DcMotor front_left = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status","Initialized");
        telemetry.update();

        front_left = hardwareMap.get(DcMotor.class,"front_left");
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setDirection(DcMotor.Direction.FORWARD);

        front_right = hardwareMap.get(DcMotor.class,"front_right");
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setDirection(DcMotor.Direction.REVERSE);

        back_right = hardwareMap.get(DcMotor.class,"back_right");
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        back_left = hardwareMap.get(DcMotor.class,"back_left");
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setDirection(DcMotor.Direction.FORWARD);



        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

        waitForStart();

        while (opModeIsActive()) {

            back_left.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
            front_left.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
            front_right.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
            back_right.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);

        }

    }

}


