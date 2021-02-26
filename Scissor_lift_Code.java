package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="Scissor_lift_Code", group="Linear Opmode")
public class Scissor_lift_Code extends LinearOpMode {

        private DcMotor right_lift = null;
        private DcMotor left_lift = null;


    @Override
    public void runOpMode()  {
        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotor.Direction.FORWARD);

        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setDirection(DcMotor.Direction.REVERSE);

        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();


        while (opModeIsActive()) {

           if (gamepad1.a) {
               right_lift.setTargetPosition(230);
               left_lift.setTargetPosition(230);

               right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

               right_lift.setPower(0.25);
               left_lift.setPower(0.25);

               telemetry.addData("position", right_lift.getCurrentPosition());
               telemetry.addData("position", left_lift.getCurrentPosition());
               telemetry.addData("is at target", right_lift.isBusy());
               telemetry.addData("is at target", left_lift.isBusy());
               telemetry.update();
           }

            if (gamepad1.b) {
                right_lift.setTargetPosition(-30);
                left_lift.setTargetPosition(-30);

                right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                right_lift.setPower(0.25);
                left_lift.setPower(0.25);

                telemetry.addData("position", right_lift.getCurrentPosition());
                telemetry.addData("position", left_lift.getCurrentPosition());
                telemetry.addData("is at target", right_lift.isBusy());
                telemetry.addData("is at target", left_lift.isBusy());
                telemetry.update();
            }

        }

    }
}
