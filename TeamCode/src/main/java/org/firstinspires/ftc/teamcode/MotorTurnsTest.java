package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTurnsTest extends LinearOpMode {

    private DcMotor backLeftMotor;

    @Override
    public void runOpMode() {

        backLeftMotor = hardwareMap.get(DcMotor.class, "motor0");

        waitForStart();

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset encoder count kept by left motor.
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {

            if(gamepad1.a) {
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            backLeftMotor.setPower(gamepad1.left_stick_y);
            int ticks = backLeftMotor.getCurrentPosition();

            telemetry.addData("Ticks ", ticks);
            telemetry.update();
        }

        backLeftMotor.setPower(0);
    }
}
