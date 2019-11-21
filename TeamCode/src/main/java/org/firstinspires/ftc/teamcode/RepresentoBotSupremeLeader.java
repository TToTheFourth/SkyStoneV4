package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RepresentoBotSupremeLeader {

    private Timer myTimer;

    private DcMotor backLeftMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor slideMotorF;
    private DcMotor trackMotorF;
    private Servo foundationServo;
    private Servo stoneServo;
    private Gyro gyro;
    private LinearOpMode opMode;
    ModernRoboticsI2cRangeSensor rangeSensor;

    public RepresentoBotSupremeLeader(LinearOpMode om) {
        this.opMode = om;


        //backLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        //frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        //frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        //backRightMotor = hardwareMap.get(DcMotor.class, "motor3");

        backLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor0");
        frontLeftMotor = opMode.hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor2");
        backRightMotor = opMode.hardwareMap.get(DcMotor.class, "motor3");//slideMotorF = opMode.hardwareMap.get(DcMotor.class, "motor4");
        //trackMotorF = opMode.hardwareMap.get(DcMotor.class, "motor5");
        //foundationServo = opMode.hardwareMap.get(Servo.class, "foundServo");
        BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro = new Gyro(imu, opMode);
        myTimer = new Timer();
        //stoneServo = opMode.hardwareMap.get(Servo.class, "stoneServo");

        //stops movement of robot quickly.
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slideMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //trackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startGyro(){
        gyro.StartGyro();

    }

    public void turnRight(double degrees, double power) {
        gyro.resetWithDirection(Gyro.RIGHT);

        // start the motors turning right
        double rightY_G1 = 0.0;
        double rightX_G1 = -1.0 * power;
        double leftX_G1 = 0.0;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);

        // loop until the robot turns :) degrees
        double d = -1 * degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() <= d) {
                break;
            }
        }

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);

    }

    public void turnLeft(double degrees, double power) {
        gyro.resetWithDirection(Gyro.LEFT);

        // start the motors turning left
        double rightY_G1 = 0.0;
        double rightX_G1 = 1.0 * power;
        double leftX_G1 = 0.0;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);

        // loop until the robot turns :) degrees
        double d = degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() >= d) {
                break;
            }
        }

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);

    }

    double factor = 1.0;
    public void goForward(double power, double distance){


        double rightY_G1 = 1.0 * power;
        double rightX_G1 = 0.0;
        double leftX_G1 = 0.0;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);

        myTimer.setCompareTime(inchesToTime(distance, power));
        myTimer.start();
        while (opMode.opModeIsActive()) {
            // TODO: need brackets for the if and a break statement
            if (myTimer.timeChecker()) {
                break;
            }
        }

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
    }

    public long inchesToTime(double inches, double power) {
        return (long) (0.0384 * inches * 500.0 / power);
    }
}
