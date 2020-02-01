import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RepresentoBotSupremeLeader;

@Autonomous
public class RackBack extends LinearOpMode {


    @Override
    public void runOpMode() {

        RepresentoBotSupremeLeader bot = new RepresentoBotSupremeLeader(this);
        waitForStart();

        bot.timeRackIn(3.5f);
    }
}