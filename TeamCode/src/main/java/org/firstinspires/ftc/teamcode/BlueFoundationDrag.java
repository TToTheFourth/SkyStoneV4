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
        bot.goForward(-0.5, 1);
        bot.slide(-0.5, 4);
        bot.goForward(-0.5, 30);
        bot.servoLatch();
        timer.waitT(1000);
        bot.goForward(0.5, 31);
        bot.slide(-0.5, 4.5);
        bot.goForward(0.5, 4.5);
        timer.waitT(1000);
        bot.ServoUnlatch();
        timer.waitT(1000);
        bot.slide(0.75, 60);
    }
}
