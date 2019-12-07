package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

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
    private Servo left;
    private Servo right;
    private Gyro gyro;
    private LinearOpMode opMode;
    private DistanceSensor sensorDistance;
    private NormalizedColorSensor sensorColor;
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
        left = opMode.hardwareMap.get(Servo.class, "left");
        right = opMode.hardwareMap.get(Servo.class, "right");
        sensorDistance = opMode.hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorColor = opMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
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
        //
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
    public void servoLatch() {
        right.setPosition(1.0);
        left.setPosition(1.0);
    }

    public void ServoUnlatch() {
        right.setPosition(0);
        left.setPosition(0);
    }

    public void forwardUntil(double until, double power) {
        double rightY_G1 = 1.0 * power;
        double rightX_G1 = 0.0;
        double leftX_G1 = 0.0;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);

        while (opMode.opModeIsActive()) {
            if (sensorDistance.getDistance(DistanceUnit.INCH) < until) {
                break;
            }
        }

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
    }

    public void slide (double power, double distance) {
        double rightY_G1 = 0.0;
        double rightX_G1 = 0.0;
        double leftX_G1 = 1.0 * power;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long ticks = ticksToInchesSlide(distance);
        while (opMode.opModeIsActive()) {
            if (backLeftMotor.getCurrentPosition() >= ticks) {
                break;
            }
        }
    }

    public void slideWhile(double power) {
        double rightY_G1 = 0.0;
        double rightX_G1 = 0.0;
        double leftX_G1 = 1.0 * power;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);

        float hsvValues[] = {0, 0, 0};
        while (opMode.opModeIsActive()) {
            getColor(hsvValues);
            float h = hsvValues[0];
            float s = hsvValues[1];
            float v = hsvValues[2];
            if (!(37.1 < h && 45.1 > h && 0.5 < s && 0.651 > s && 0.433 < v && 0.457 > v)) {
                break;
            }
            /*
            H: 45.1 - 37.1
            S: 0.651 - 0.500
            V: 0.457 - 0.433
            */
        }

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
    }

    public void getColor(float [] hsvValues) {
        NormalizedRGBA colors = sensorColor.getNormalizedColors();
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;

        Color.colorToHSV(colors.toColor(), hsvValues);
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

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long ticks = ticksToInchesForward(distance);
        while (opMode.opModeIsActive()) {
            int rotations = backLeftMotor.getCurrentPosition();
            if (rotations<0) {
                rotations = rotations * -1;
            }
            if (rotations >= ticks) {
                break;
            }
        }

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
    }
    public long ticksToInchesForward(double inches) {
        return (long) (inches * 30.26086956217);
    }
    public long ticksToInchesSlide(double inches) {
        return (long) (inches * 31.930232558139);
    }
}
