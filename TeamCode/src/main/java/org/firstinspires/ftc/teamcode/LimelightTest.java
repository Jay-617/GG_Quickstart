package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Limelight Test")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addLine("Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (limelight != null) {
                telemetry.addLine("Limelight is connected!");
            } else {
                telemetry.addLine("Limelight NOT FOUND!");
            }
            telemetry.update();
        }
    }
}
