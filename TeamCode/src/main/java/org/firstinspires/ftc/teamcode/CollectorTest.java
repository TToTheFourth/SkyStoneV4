package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.Directions;
import org.firstinspires.ftc.teamcode.vision.VuHolder;

@Autonomous
public class CollectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        VuHolder vu = new VuHolder(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(!this.isStopRequested()) {
            if (vu.check() == true) {
                Directions dir = vu.getDirections(35, 35);
                telemetry.addData("heading %.1f", dir.getHeading());
                telemetry.addData("distance %.1f", dir.getDistance());
               // telemetry.addData("Working", "YES");
                telemetry.update();
            } else {
               // telemetry.addData("Working", "NO");
               // telemetry.update();
            }
        }
    }
}
