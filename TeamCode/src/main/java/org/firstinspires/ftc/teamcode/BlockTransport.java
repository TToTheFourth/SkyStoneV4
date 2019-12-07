package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlockTransport extends LinearOpMode {
    @Override
    public void runOpMode() {


        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        bot.startGyro();
        bot.forwardUntil(2, 0.2);
        bot.slideWhile(0.2);
    }
}