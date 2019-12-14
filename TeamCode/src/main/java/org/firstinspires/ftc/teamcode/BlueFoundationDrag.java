package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueFoundationDrag extends LinearOpMode {
    @Override
    public void runOpMode() {

        Timer timer = new Timer();
        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        bot.startGyro();
        bot.ServoUnlatch();
        //resets servo position
        timer.waitT(8000);
        // waits in case alliance partner wants to place a block on the foundation
        bot.goForward(-0.5, 1);
        // moves off wall
        bot.slide(-0.5, 4);
        // repositions to face the middle of the foundation
        bot.goForward(-0.5, 30);
        // move to the foundation
        bot.servoLatch();
        // latches the servos on the foundation
        timer.waitT(1000);
        // makes sure the servos latched unto the foundation
        bot.goForward(0.5, 31);
        // moves back to the wall
        bot.slide(-0.5, 4.5);
        // slides into building corner
        bot.goForward(0.5, 6);
        // pulls foundation all the way back
        timer.waitT(1000);
        // makes sure the robot is done moving
        bot.ServoUnlatch();
        // unlatches the robot from the foundation
        timer.waitT(1000);
        // makes sure the servo has unlatched from the foundation
        bot.slide(0.75, 53);
        // slides unto the line
    }
}
