package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MecanumWheelTestControls extends LinearOpMode {

    private DcMotor backLeftMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;

    @Override
    public void runOpMode() {

        backLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double tgtPowerLB = 0;
        double tgtPowerRB = 0;
        double tgtPowerLF = 0;
        double tgtPowerRF = 0;
        double factor = 2;

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

                double rightX_G1;
                double rightY_G1;
                double leftX_G1;
                double leftY_G1;
                double leftX_G2;
                double leftY_G2;
                double rightX_G2;
                double rightY_G2;

                rightX_G1 = gamepad1.right_stick_x;
                rightY_G1 = -gamepad1.right_stick_y;
                leftX_G1 = -gamepad1.left_stick_x;
                leftY_G1 = -gamepad1.left_stick_y;
                leftX_G2 = gamepad2.left_stick_x;
                leftY_G2 = gamepad2.left_stick_y;
                rightX_G2 = gamepad2.right_stick_x;
                rightY_G2 = gamepad2.right_stick_y;

                if (gamepad1.y) {
                    factor = 1;
                } else if (gamepad1.a) {
                    factor = 3;
                } else if (gamepad1.x) {
                    factor = 2;
                }


                frontLeftMotor.setPower((leftY_G1 + rightX_G1 - leftX_G1) / factor);
                backLeftMotor.setPower((leftY_G1 + rightX_G1 + leftX_G1) / factor);
                backRightMotor.setPower((leftY_G1 - rightX_G1 + leftX_G1) / factor);
                frontRightMotor.setPower((leftY_G1 - rightX_G1 - leftX_G1) / factor);

        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
