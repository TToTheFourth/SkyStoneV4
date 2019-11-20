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
        double multiplier = 1;
        boolean wheelsOn;

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

                rightX_G1 = gamepad1.right_stick_y;
                rightY_G1 = -gamepad1.right_stick_x;
                leftX_G1 = -gamepad1.left_stick_y;
                leftY_G1 = -gamepad1.left_stick_x;
                leftX_G2 = gamepad2.left_stick_y;
                leftY_G2 = gamepad2.left_stick_x;
                rightX_G2 = gamepad2.right_stick_y;
                rightY_G2 = gamepad2.right_stick_x;

                if (gamepad1.y) {
                    factor = 1;
                } else if (gamepad1.a) {
                    factor = 3;
                } else if (gamepad1.x) {
                    factor = 2;
                }

                if(gamepad1.left_stick_y > 0) {
                    wheelsOn = true;
                } else if(gamepad1.left_stick_x > 0) {
                    wheelsOn = true;
                } else {
                    wheelsOn = false;
                }

//                 if NOPOWER and power applied then state = ACCELERATE, percent = .1
//                 if ACCELERATE and power applied and percent < 1 then percent = percent + .1
//                 if ACCELERATE and power applied and percent >= 1 then state = STEADY
//                 if power not applied then state = NOPOWER
//                 if state = STEADY then state = STEADY
//                 set power (power * percent)

                if (wheelsOn = true) {
                    for (int i = 0; i <= 10; i++) {
                        if (i > 1) {
                            multiplier = 0.1;
                        } else if (i > 2 && i < 4) {
                            multiplier = 0.2;
                        } else if (i > 4 && i < 6) {
                            multiplier = 0.4;
                        } else if (i > 6 && i < 8) {
                            multiplier = 0.6;
                        } else if (i > 8 && i < 10) {
                            multiplier = 0.8;
                        } else if (i > 10) {
                            multiplier = 1;
                            break;
                        }
                    }
                }


                frontLeftMotor.setPower(((leftY_G1 + rightX_G1 - leftX_G1) / factor) * multiplier);
                backLeftMotor.setPower(((leftY_G1 + rightX_G1 + leftX_G1) / factor) * multiplier);
                backRightMotor.setPower(((leftY_G1 - rightX_G1 + leftX_G1) / factor) * multiplier);
                frontRightMotor.setPower(((leftY_G1 - rightX_G1 - leftX_G1) / factor) * multiplier);


        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
