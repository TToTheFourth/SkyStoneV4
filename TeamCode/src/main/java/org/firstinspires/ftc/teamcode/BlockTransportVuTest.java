package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.Directions;
import org.firstinspires.ftc.teamcode.vision.VuHolder;

@Disabled
@Autonomous
public class BlockTransportVuTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        Timer timer = new Timer();
        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);
        VuHolder vu = new VuHolder(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        bot.getReady(.9);
       // vu.activate();
        bot.startGyro();
       // vu.check();
        if(vu.check() == true) {
            Directions dir = vu.getDirections(-36,70 );
            telemetry.addData("YES", "Working");
            telemetry.update();
        } else {
            telemetry.addData("NO", "Working");
            telemetry.update();
        }
       // vu.deactivate();
    }
}