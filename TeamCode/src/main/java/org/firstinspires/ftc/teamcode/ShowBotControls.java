package org.firstinspires.ftc.teamcode;


import android.view.Display;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ShowBotControls extends LinearOpMode {

    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;

    @Override
    public void runOpMode() {
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double tgtPowerLB = 0;
        double tgtPowerRB = 0;
        double tgtPowerLF = 0;
        double tgtPowerRF = 0;
        double factor = 2;

        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            tgtPowerLB = gamepad1.left_stick_y;
            tgtPowerRF = -gamepad1.right_stick_y;
            tgtPowerLF = gamepad1.left_stick_y;
            tgtPowerRB = -gamepad1.right_stick_y;

            if(gamepad1.y){
                factor = 1;
            }else if(gamepad1.a) {
                factor = 3;
            }else if(gamepad1.x){
                factor = 2;
            }

            leftMotorF.setPower(tgtPowerLF/factor);
            leftMotorB.setPower(tgtPowerLB/factor);
            rightMotorF.setPower(tgtPowerRF/factor);
            rightMotorB.setPower(tgtPowerRB/factor);

            telemetry.addData("Target Power Left Back", tgtPowerLB);
            telemetry.addData("Target Power Right Back", tgtPowerRB);
            telemetry.addData("Target Power Left Front", tgtPowerLF);
            telemetry.addData("Target Power Right Front", tgtPowerRF);

        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

    }

}
