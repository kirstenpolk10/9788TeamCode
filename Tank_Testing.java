package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Tank_Testing", group="Linear Opmode")
public class Tank_Testing extends LinearOpMode {


    static final double     TICKS_PER_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    /*static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;*/

    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor right_lift = null;
    private DcMotor left_lift = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setDirection(DcMotor.Direction.FORWARD);

        right_lift = hardwareMap.get(DcMotor.class, "right_lift");
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setDirection(DcMotor.Direction.FORWARD);

        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setDirection(DcMotor.Direction.REVERSE);


        back_left.setPower(0);
        back_right.setPower(0);



        /*left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path0",  "Starting at %7d :%7d",
                left_lift.getCurrentPosition(),
                right_lift.getCurrentPosition());
        telemetry.update();

        double quarter_turn =  TICKS_PER_REV/4;

        left_lift.setTargetPosition((int)TICKS_PER_REV*5);
        right_lift.setTargetPosition((int)TICKS_PER_REV*5);*/

        waitForStart();


        while (opModeIsActive()) {


            /*left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

            double leftPower;
            double rightPower;

            double drive = -gamepad1.right_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            back_left.setPower(leftPower);
            back_right.setPower(rightPower);


            left_lift.setPower(gamepad1.left_stick_y);
            right_lift.setPower(gamepad1.left_stick_y);

                /*left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/




        }

    }

}
