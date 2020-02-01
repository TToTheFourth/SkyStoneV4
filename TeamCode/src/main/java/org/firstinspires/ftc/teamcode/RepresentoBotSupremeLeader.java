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
import org.firstinspires.ftc.teamcode.vision.VuHolder;

import java.util.ArrayList;
import java.util.List;

public class RepresentoBotSupremeLeader {

    private Timer myTimer;

    private DcMotor backLeftMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor rackMotor;
    private Servo claw;
    private Servo servoCon;

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
    private Gyro2 miniGyro;

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
        rackMotor = opMode.hardwareMap.get(DcMotor.class, "rackmotor");
        servoCon = opMode.hardwareMap.get(Servo.class, "servoCon");
        claw = opMode.hardwareMap.get(Servo.class, "claw");
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
        rackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slideMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //trackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startGyro(){
        gyro.StartGyro();
        miniGyro = gyro.getMiniGyro();
    }

    public void turnRight(double degrees, double power) {
        //
        gyro.resetWithDirection(Gyro.RIGHT);
        // tells the gyro we are turning right

        // start the motors turning right
        double rightY_G1 = 0.0;
        double rightX_G1 = -1.0 * power;
        double leftX_G1 = 0.0;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);
        // connects the motors to the correct variables

        // loop until the robot turns :) degrees
        double d = -1 * degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() <= d) {
                break;
            }
        }
        // gets angle turn

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        // stops the motors
    }

    public void turnLeft(double degrees, double power) {
        gyro.resetWithDirection(Gyro.LEFT);
        // tells gyro we are going left

        // start the motors turning left
        double rightY_G1 = 0.0;
        double rightX_G1 = 1.0 * power;
        double leftX_G1 = 0.0;

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);
        // sets the motors to the correct variables

        // loop until the robot turns :) degrees
        double d = degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() >= d) {
                break;
            }
        }
        // gets degrees

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        // stops motors
    }
    public void servoLatch() {
        right.setPosition(1.0);
        left.setPosition(1.0);
        // latches both servos, moving them to the down position
    }

    public void ServoUnlatch() {
        right.setPosition(0);
        left.setPosition(0);
        // unlatches both servos, moving them to the up position
    }

    public void forwardUntil(double until, double power) {
        double rightY_G1 = 1.0 * power;
        double rightX_G1 = 0.0;
        double leftX_G1 = 0.0;
        // sets power

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);
        // connects motors to the correct variable(s)

        miniGyro.reset();
        while (opMode.opModeIsActive()) {
            if (sensorDistance.getDistance(DistanceUnit.INCH) < until) {
                break;
            }

            if (miniGyro.getAngle() > 4) {
                turnRight(0.75 * miniGyro.getAngle(), 0.3);
            }else if (miniGyro.getAngle() < -4){
                turnLeft (-0.75 * miniGyro.getAngle(), 0.3);
            }
        }
        // sets the distance sensor to go until we are inches away from something

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        // stops motor
    }

    public void slide (double power, double distance) {
        double rightY_G1 = 0.0;
        double rightX_G1 = 0.0;
        double leftX_G1 = 1.0 * power;
        // sets power

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);
        // connects power to the correct variables

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // sets encoder

        miniGyro.reset();
        long ticks = ticksToInchesSlide(distance);
        while (opMode.opModeIsActive()) {
            int rotations = backLeftMotor.getCurrentPosition();
            if (rotations<0) {
                rotations = rotations * -1;
            }
            if (rotations >= ticks) {
                break;
            }

            if (miniGyro.getAngle() > 4) {
                turnRight(0.75 * miniGyro.getAngle(), 0.3);
            }else if (miniGyro.getAngle() < -4){
                turnLeft (-0.75 * miniGyro.getAngle(), 0.3);
            }
        }
        // sets the inches to ticks so the motors understand
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void slideWhile(double power) {
        double rightY_G1 = 0.0;
        double rightX_G1 = 0.0;
        double leftX_G1 = 1.0 * power;
        // sets power

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);
        // sets variables to the correct variables

        float hsvValues[] = {0, 0, 0};
        while (opMode.opModeIsActive()) {
            getColor(hsvValues);
            float h = hsvValues[0];
            float s = hsvValues[1];
            float v = hsvValues[2];
            if (!(35.1 < h && 47.1 > h && 0.4 < s && 0.751 > s && 0.333 < v && 0.557 > v)) {
                break;
            }
            // tells the sensor what the hsv values need to be before it needs to stop
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
        // stops the motors
    }

    public void getColor(float [] hsvValues) {
        NormalizedRGBA colors = sensorColor.getNormalizedColors();
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        // normalizes colors

        Color.colorToHSV(colors.toColor(), hsvValues);
        // tells the colors now that they are normalized
    }

    public void timeRackIn(float time) {
        myTimer.setCompareTime((long)(time * 1000));
        rackMotor.setPower(1);
        myTimer.start();
        while (opMode.opModeIsActive()) {
            if (myTimer.timeChecker()) {
                break;
            }
        }
        rackMotor.setPower(0);
        // moves the rack for a certain amount of time
    }

    public void timeRackOut(float time) {
        myTimer.setCompareTime((long)(time * 1000));
        rackMotor.setPower(-1);
        myTimer.start();
        while (opMode.opModeIsActive()) {
            if (myTimer.timeChecker()) {
                break;
            }
        }
        rackMotor.setPower(0);
        // moves the rack for a certain amount of time
    }

    public void moveClaw(double position) {
        servoCon.setPosition(position);
        // sets claw to a position
    }

    public void getReady(double pos) {
        claw.setPosition(pos);
        myTimer.waitT(1000);
    }

    double factor = 1.0;
    public void goForward(double power, double distance){
        double rightY_G1 = 1.0 * power;
        double rightX_G1 = 0.0;
        double leftX_G1 = 0.0;
        // sets power

        frontLeftMotor.setPower((rightX_G1 + rightY_G1 - leftX_G1) / factor);
        backLeftMotor.setPower((rightX_G1 + rightY_G1 + leftX_G1) / factor);
        backRightMotor.setPower((rightX_G1 - rightY_G1 + leftX_G1) / factor);
        frontRightMotor.setPower((rightX_G1 - rightY_G1 - leftX_G1) / factor);
        // sets the correct variables to the motors

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // sets the encoders

        long ticks = ticksToInchesForward(distance);
        miniGyro.reset();
        while (opMode.opModeIsActive()) {
            int rotations = backLeftMotor.getCurrentPosition();
            if (rotations<0) {
                rotations = rotations * -1;
            }
            if (rotations >= ticks) {
                break;
            }

            if (miniGyro.getAngle() > 4) {
                turnRight(0.75 * miniGyro.getAngle(), 0.3);
            }else if (miniGyro.getAngle() < -4){
                turnLeft (-0.75 * miniGyro.getAngle(), 0.3);
            }
            // makes inches transfer to ticks
        }

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        // sets motors to zero
    }
    public void stopMotor(){
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
    }
    public long ticksToInchesForward(double inches) {
        return (long) (inches * 30.26086956217);
        // ticks forward formula
    }
    public long ticksToInchesSlide(double inches) {
        return (long) (inches * 31.930232558139);
        // tick to slide inches formula
    }
    public long inchesToTime(double inches, double power) {
        return (long) (0.0384 * inches * 500.0 / power);
        // inches to time formula
    }
}
