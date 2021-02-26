package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


    @TeleOp(name="GravityArm", group="Linear Opmode")
    public class GravityArm extends LinearOpMode {

        private DcMotor front_right = null;


        @Override
        public void runOpMode() {
            telemetry.addData("Status","Initialized");
            telemetry.update();

            front_right = hardwareMap.get(DcMotor.class,"front_right");
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setDirection(DcMotor.Direction.REVERSE);

            front_right.setPower(0);

            waitForStart();

            while (opModeIsActive()) {

                front_right.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);


            }

        }

    }
