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
        //the robot goes forward until the range sensor is close to the blocks,
        // so we don't have to rely on how many inches we need to go
        bot.moveClaw(1);
        // puts claw down on the first block it interacts with ( usually first or second)
        timer.waitT(2000);
        // gives time for the claw to fully grab the block
        bot.goForward(-0.5, 26);
        // backs up towards the wall with block
        bot.turnRight(80, 0.5 );
        // turns to face the building site
        bot.goForward(0.5,50);
        // goes across the line to building site
        bot.moveClaw(0);
        // releases the block
        bot.goForward(0.5, -8);
        // todo: is needed?
        // moves to stop on line
        bot.goForward(-0.5, 20);
        // moves to stop on line
    }
}