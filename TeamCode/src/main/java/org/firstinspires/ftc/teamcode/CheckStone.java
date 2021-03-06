package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.VuHolder;

@Disabled
@Autonomous
public class CheckStone extends LinearOpMode {
    @Override
    public void runOpMode() {

        VuHolder vu = new VuHolder(this);

        waitForStart();
        vu.activate();

        while(!isStopRequested()) {
            if (vu.checkForSkystone()) {
                telemetry.addData("Working?", "Yes");
                telemetry.update();
            } else {
                telemetry.addData("Working?", "NO");
                telemetry.update();
            }
        }
    }
}
