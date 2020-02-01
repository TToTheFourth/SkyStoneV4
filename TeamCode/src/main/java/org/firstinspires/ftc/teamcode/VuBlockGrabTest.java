package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.vision.Directions;
import org.firstinspires.ftc.teamcode.vision.VuHolder;

@Autonomous
public class VuBlockGrabTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        VuHolder vu = new VuHolder(this);
        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);
        float path[][] = {{-35, -45, 3.14f / 2}, {-15, -45, 3.14f / 2}, {0, -45, 3.14f / 2}, {15, -40, 3.14f / 2}, {35, -35, 3.14f / 2}, {35, 15, 3.14f / 2}, {35, 0, 3.14f / 2}, {35, 15, 3.14f / 2}, {35, 35, 3.14f / 2}};

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
            //bot.getReady(0.9);
            bot.timeRackOut(4f);
            CameraDevice.getInstance().setFlashTorchMode(true);
            bot.forwardUntil(10, 0.3);
//            if (!vu.checkForSkystone()) {
//                vu.checkForSkystone();
                for (int g = 0; g < 11; g++) {
                    if (vu.checkForSkystone()) {
                        bot.slide(0.3, 8);
                        break;
                    }
                    bot.slide(0.5, 4);
                    sleep(500);
                }
            bot.goForward(0.5, 5);
                bot.getReady(.3);
                bot.goForward(-0.5, 7.5);
                while (!vu.check()) {
                    bot.turnRight(10, 0.4);
                }
                    Directions dirt = vu.getDirections(45, -45);
                    float dirst = (float) Math.toDegrees(dirt.getHeading());
                    float dist = dirt.getDistance();
                    if (dirst > 0) {
                        bot.turnLeft(dirst, 0.3);
                    } else {
                        bot.turnRight(-dirst, 0.3);
                    bot.goForward(-0.5, 10);
                    bot.getReady(.9);
                    while (!vu.check()) {
                        bot.turnRight(10, 0.4);
                    }
//                        vu.getDirections(-56, -45);
//                        dirst = (float) Math.toDegrees(dirt.getHeading());
//                        dist = dirt.getDistance();
//                        if (dirst > 0) {
//                            bot.turnLeft(dirst, 0.3);
//                        } else {
//                            bot.turnRight(-dirst, 0.3);
//                        }
//                        bot.goForward(0.5, dist);
//                        bot.forwardUntil(2, 0.3);
//                        bot.slideWhile(0.3);
//                        bot.getReady(.3);
//                        dirt = vu.getDirections(45, -45);
//                        dirst = (float) Math.toDegrees(dirt.getHeading());
//                        dist = dirt.getDistance();
//                        if (dirst > 0) {
//                            bot.turnLeft(dirst, 0.3);
//                        } else {
//                            bot.turnRight(-dirst, 0.3);
//                        }
//                        bot.goForward(0.5, dist);
//                        bot.getReady(.9);
                    dirt = vu.getDirections(0, -50);
                    dirst = (float) Math.toDegrees(dirt.getHeading());
                    dist = dirt.getDistance();
                    if (dirst > 0) {
                        bot.turnLeft(dirst, 0.3);
                    } else {
                        bot.turnRight(-dirst, 0.3);
                    }
                    bot.goForward(0.5, dist);
            }

//             {
//                telemetry.addData("Working", "NO");
//                telemetry.update();
            }
//                bot.timeRackOut(4.5f);
            bot.stopMotor();
       // }
        bot.stopMotor();
        vu.deactivate();
    }
}
