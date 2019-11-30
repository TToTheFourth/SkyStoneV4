import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@TeleOp
public class ServoTest  extends LinearOpMode {


    @Override
    public void runOpMode() {

        CRServo slideWind = hardwareMap.get(CRServo.class, "slide_wind");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            slideWind.setPower(1);

        }

        slideWind.setPower(0);

    }
}