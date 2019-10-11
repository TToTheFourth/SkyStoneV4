package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RackNServo extends LinearOpMode {
    private DcMotor rackMotor;
    private Servo squeezeServo;

    @Override
    public void runOpMode() {
        rackMotor = hardwareMap.get(DcMotor.class, "motor0");
        squeezeServo = hardwareMap.get(Servo.class, "servo0");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double tgtPowerRM = 0;
        double tgtPowerSqueezeServo = 0;

        rackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            tgtPowerRM = -gamepad1.left_stick_y;
            tgtPowerSqueezeServo = this.gamepad1.right_stick_y;

            rackMotor.setPower(tgtPowerRM);
            squeezeServo.setPosition(tgtPowerSqueezeServo);

            telemetry.addData("Target Power Rack Motor", tgtPowerRM);
            telemetry.addData("Target Power Squeeze Servo", tgtPowerSqueezeServo);

        }

        rackMotor.setPower(0);
        squeezeServo.setPosition(0);
    }

}
