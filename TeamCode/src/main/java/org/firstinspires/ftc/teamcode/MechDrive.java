package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@TeleOp
public class MechDrive extends LinearOpMode {

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
        CRServo slideWind = hardwareMap.get(CRServo.class, "slide_wind");
        DcMotor rackmotor = hardwareMap.get(DcMotor.class, "rackmotor");
        Servo claw =hardwareMap.get(Servo.class, "claw");
        telemetry.addData("Status", "Initialized");
        Servo left =hardwareMap.get(Servo.class, "left");
        Servo right =hardwareMap.get(Servo.class, "right");
        telemetry.update();

        waitForStart();

        double factor = 2;
        double multiplier = 1;

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(0.5);
        while (opModeIsActive()) {

                double rightX_G1;
                double rightY_G1;
                double leftX_G1;
                double leftY_G1;
                double leftX_G2;
                double leftY_G2;
                double rightX_G2;
                double rightY_G2;

                rightY_G1 = -gamepad1.right_stick_y;
                rightX_G1 = -gamepad1.right_stick_x;
                leftY_G1 = gamepad1.left_stick_y;
                leftX_G1 = -gamepad1.left_stick_x;
                leftY_G2 = gamepad2.left_stick_y;
                leftX_G2 = gamepad2.left_stick_x;
                rightY_G2 = gamepad2.right_stick_y;
                rightX_G2 = gamepad2.right_stick_x;


                if(gamepad1.dpad_up ==true) {
                    slideWind.setPower(1);
                } else if(gamepad1.dpad_down == true) {
                    slideWind.setPower(-1);
                } else {
                    slideWind.setPower(0);
                }


                if(gamepad1.dpad_left ==true) {
                    rackmotor.setPower(1.0);
                } else if(gamepad1.dpad_right == true) {
                    rackmotor.setPower(-1.0);
                } else {
                    rackmotor.setPower(0);
                }

                //0.075
                double clawPos = gamepad1.right_trigger;
                if(clawPos < 0.075) {
                    clawPos = 0.075;
                }
                claw.setPosition(clawPos);

                if (gamepad1.right_bumper == true) {
                    right.setPosition(1.0);
                    left.setPosition(1.0);
                }
                else {
                    right.setPosition(0);
                    left.setPosition(0);
                }

                if (gamepad1.y) {
                    factor = 1;
                } else if (gamepad1.a) {
                    factor = 3;
                } else if (gamepad1.x) {
                    factor = 2;
                }

                frontLeftMotor.setPower(((rightX_G1 + rightY_G1 - leftX_G1) / factor) * multiplier);
                backLeftMotor.setPower(((rightX_G1 + rightY_G1 + leftX_G1) / factor) * multiplier);
                backRightMotor.setPower(((rightX_G1 - rightY_G1 + leftX_G1) / factor) * multiplier);
                frontRightMotor.setPower(((rightX_G1 - rightY_G1 - leftX_G1) / factor) * multiplier);

        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
