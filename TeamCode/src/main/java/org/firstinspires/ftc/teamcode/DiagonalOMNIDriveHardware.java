package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DiagonalOMNIDriveHardware {

    /**
     * This is NOT an opmode.
     *
     * This class can be used to define all the specific hardware for a single robot.
     * In this case that robot is a FTC RED'S SICK MECANUM DRIVEN DEATHMACHINE.
     * See your mom and other classes starting with "Not A Phase" for usage examples.
     *
     * This hardware class assumes the following device names have been configured on the robot:
     * Note:  All names are lower case and some have single spaces between words.
     *
     * Motor channel:  Left  drive motor:        "left_drive"
     * Motor channel:  Right drive motor:        "right_drive"
     * Motor channel:  Manipulator drive motor:  "left_arm"
     * Servo channel:  Servo to open left claw:  "left_hand"
     * Servo channel:  Servo to open right claw: "right_hand"
     */

    /* Public OpMode members. */
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */
    public DiagonalOMNIDriveHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        backLeftMotor = ahwMap.get(DcMotor.class, "motor0");
        frontLeftMotor = ahwMap.get(DcMotor.class, "motor1");
        frontRightMotor = ahwMap.get(DcMotor.class, "motor2");
        backRightMotor = ahwMap.get(DcMotor.class, "motor3");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        // Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}