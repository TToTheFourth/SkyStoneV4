//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class TheDriveClass{
//
//    public void forward() { {
//
//        double rightX_G1;
//        double rightY_G1;
//        double leftX_G1;
//        double leftY_G1;
//        double leftX_G2;
//        double leftY_G2;
//        double rightX_G2;
//        double rightY_G2;
//
//        double factor = 2;
//        double tgtPowerLB = leftY_G1 + rightX_G1 + leftX_G1;
//        double tgtPowerRB = leftY_G1 - rightX_G1 + leftX_G1;
//        double tgtPowerLF = leftY_G1 + rightX_G1 - leftX_G1;
//        double tgtPowerRF = leftY_G1 - rightX_G1 - leftX_G1;
//
//        rightX_G1 = 0.5;
//        rightY_G1 = -0.5;
//        leftX_G1 = 0;
//        leftY_G1 = 0;
//        leftX_G2 = 0;
//        leftY_G2 = 0;
//        rightX_G2 = 0.5;
//        rightY_G2 = -0.5;
//
//        frontLeftMotor.setPower(tgtPowerLF / factor);
//        backLeftMotor.setPower(tgtPowerLB / factor);
//        backRightMotor.setPower(tgtPowerRB / factor);
//        frontRightMotor.setPower(tgtPowerRF / factor);
//
//    }
//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//    }
//}