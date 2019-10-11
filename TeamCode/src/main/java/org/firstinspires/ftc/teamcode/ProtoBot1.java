package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ProtoBot1 extends LinearOpMode {

    private DcMotor rackMotor;

    @Override
    public void runOpMode() {
        rackMotor = hardwareMap.get(DcMotor.class, "rackMotor");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        rackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

          double  rackPower = gamepad1.left_stick_y; //5

            rackMotor.setPower (rackPower);

            telemetry.addData("Target rack Power", rackPower);
            telemetry.update();
        }

        rackMotor.setPower(0);
    }

}
