package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RepresentoBotSupremeLeader {

    private Timer myTimer;
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;
    private DcMotor slideMotorF;
    private DcMotor trackMotorF;
    private Servo foundationServo;
    private Servo stoneServo;
    private Gyro gyro;
    private LinearOpMode opMode;
    ModernRoboticsI2cRangeSensor rangeSensor;

    public RepresentoBotSupremeLeader(LinearOpMode om) {
        this.opMode = om;

        leftMotorB = opMode.hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = opMode.hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = opMode.hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = opMode.hardwareMap.get(DcMotor.class, "motor3");
        slideMotorF = opMode.hardwareMap.get(DcMotor.class, "motor4");
        trackMotorF = opMode.hardwareMap.get(DcMotor.class, "motor5");
        foundationServo = opMode.hardwareMap.get(Servo.class, "foundServo");
        BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro = new Gyro(imu, opMode);
        myTimer = new Timer();
        stoneServo = opMode.hardwareMap.get(Servo.class, "stoneServo");

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startGyro(){
        gyro.StartGyro();

    }

    public void turnRight(double degrees, double power) {
        gyro.resetWithDirection(Gyro.RIGHT);

        // start the motors turning right
        double p = -1 * power;
        leftMotorF.setPower(p);
        leftMotorB.setPower(p);
        rightMotorF.setPower(p);
        rightMotorB.setPower(p);

        // loop until the robot turns :) degrees
        double d = -1 * degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() <= d) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

    }

    public void turnLeft(double degrees, double power) {
        gyro.resetWithDirection(Gyro.LEFT);

        // start the motors turning right
        double p = power;
        leftMotorF.setPower(p);
        leftMotorB.setPower(p);
        rightMotorF.setPower(p);
        rightMotorB.setPower(p);

        // loop until the robot turns :) degrees
        double d = degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() >= d) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

    }

    public void goForward(double power, double distance){
        double p = -1 * power;

        leftMotorF.setPower(p);
        leftMotorB.setPower(p);
        rightMotorF.setPower(power);
        rightMotorB.setPower(power);

        myTimer.setCompareTime(inchesToTime(distance, power));
        myTimer.start();
        while (opMode.opModeIsActive()) {
            // TODO: need brackets for the if and a break statement
            if (myTimer.timeChecker()) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }
    public long inchesToTime(double inches, double power) {
        return (long) (0.0384 * inches * 500.0 / power);
    }
}
