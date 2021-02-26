package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo_testing", group = "Linear Opmode")
    public class Servo_testing extends LinearOpMode {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    private CRServo intake = null;

    double  position = (MAX_POS-MIN_POS ) / 2; // Start at halfway position
    boolean rampUp = true;






    @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            intake = hardwareMap.get(CRServo.class, "intake");

            waitForStart();

                while(opModeIsActive()){

                    // slew the servo, according to the rampUp (direction) variable.
                    if (rampUp) {
                        // Keep stepping up until we hit the max value.
                        intake.setPower(1);
                        intake.setDirection(CRServo.Direction.FORWARD);

                        }

                    else {
                        intake.setPower(0);
                    }
                    if (gamepad1.a) {

                    }
                }

            }

        }

