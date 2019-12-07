package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlockTransport extends LinearOpMode {
    @Override
    public void runOpMode() {

        Timer timer = new Timer();
        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        bot.startGyro();
        bot.forwardUntil(2, 0.3);
        bot.moveClaw(1);
        timer.waitT(2000);
        bot.goForward(-0.5, 26);
        bot.turnRight(80, 0.5 );
        bot.goForward(0.5,50);
        bot.moveClaw(0);
        bot.goForward(0.5, -8);
        bot.goForward(-0.5, 20);
    }
}