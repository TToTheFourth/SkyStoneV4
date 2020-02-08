package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class ClawGetReady extends LinearOpMode {
    @Override
    public void runOpMode() {
        Timer timer = new Timer();
        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);
        waitForStart();
        bot.getReady(.9);
    }
}
