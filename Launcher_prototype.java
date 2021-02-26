package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Launcher_prototype", group="Linear Opmode")
public class Launcher_prototype extends LinearOpMode {

    private DcMotor launcher = null;
    private DcMotor launcher2 = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status","Initialized");
        telemetry.update();

        launcher = hardwareMap.get(DcMotor.class,"launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setDirection(DcMotor.Direction.FORWARD);

        launcher2 = hardwareMap.get(DcMotor.class,"launcher2");
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setDirection(DcMotor.Direction.REVERSE);

        launcher.setPower(0);
        launcher2.setPower(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                launcher.setPower(1);
                launcher2.setPower(-1);
            } else if (gamepad1.b){
                launcher.setPower(0);
                launcher2.setPower(0);
            }



        }

    }

}
