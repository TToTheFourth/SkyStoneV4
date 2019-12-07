package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class FoundationDrag extends LinearOpMode {
    @Override
    public void runOpMode() {


        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        bot.startGyro();
        bot.goForward(-0.5, 30);
        bot.servoLatch();
        bot.goForward(0.5, 30);
        bot.ServoUnlatch();
        bot.slide(0.75, 34);
    }
}
