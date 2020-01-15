package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.VuHolder;

public class TestCoordinate extends LinearOpMode {
    @Override
public void runOpMode() {
        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);
        VuHolder vu = new VuHolder(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(!isStopRequested()){
            if(vu.check() == true) {
                telemetry.addData("X Coordinate", vu.xCoor());
                telemetry.addData("Y Coordinate", vu.yCoor());
                telemetry.update();
            }else {
                telemetry.addData("Working", "NO");
                telemetry.update();
            }
        }
    }
}
