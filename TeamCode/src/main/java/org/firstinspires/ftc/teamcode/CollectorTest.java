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
        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);
        float path[][] = {{-35, -45, 3.14f/2}, {35, -45, 3.14f/2}, {35, 35, 3.14f/2}};

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        vu.activate();
        float x;
        float y;
        float curx;
        float curry;
        float curh;
        bot.startGyro();
        bot.getReady(.9);
        if (!this.isStopRequested()) {
            for (int i = 1; i < path.length; i++) {

                // new! search for target
                if(!vu.check()) {
                    for(int j = 0; j < 18; j++) {
                        bot.turnRight(10, 0.3);
                        sleep(500);
                        if(vu.check()) {
                            break;
                        }
                    }
                }

                if (vu.check() == true) {
                    //vu.check();
                    x = path[i][0];
                    y = path[i][1];
                    Directions dirt = vu.getDirections(x, y);
                    float dirst = (float)Math.toDegrees(dirt.getHeading());

                    telemetry.addData("FROM ", "(%.0f,%.0f) %.0f", vu.xCoor(), vu.yCoor(), (float)Math.toDegrees(vu.heading()));
                    telemetry.addData("TO ", "(%.0f,%.0f)", x, y);
                    telemetry.addData("DIR ", "Heading %.0f Dist %.0f", dirst, dirt.getDistance());

                    telemetry.update();
                    sleep(10000);

                    float dist = dirt.getDistance();
                    if(dirst > 0){
                        bot.turnLeft(dirst, 0.3);
                    }else{
                        bot.turnRight(-dirst, 0.3);
                    }
                    bot.goForward(0.5, dist);
                } else {
                    telemetry.addData("Working", "NO");
                    telemetry.update();
//                    curx = path[i - 1][0];
//                    curry = path[i - 1][1];
//                    curh = path[i - 1][2];
//                    x = path[i][0];
//                    y = path[i][1];
//                    float dist = vu.getDistance(x, y, curx, curry);
//                    float dir = vu.getDirection(x, y, curx, curry, curh, dist);
//                    dir = (float)Math.toDegrees(dir);
//                    if(dir > 0){
//                        bot.turnLeft(dir, 0.3);
//                    }else{
//                        bot.turnRight(dir, 0.3);
//                    }
//                    bot.goForward(0.5, dist);
                }
//                bot.timeRackOut(4.5f);
                bot.stopMotor();
            }
        }
        bot.stopMotor();
    }
}
