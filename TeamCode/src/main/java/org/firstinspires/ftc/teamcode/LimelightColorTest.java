package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Limelight Color Test", group = "Test")
public class LimelightColorTest extends LinearOpMode {

    Limelight3A limelight;

    boolean purpleDetected = false;
    boolean greenDetected = false;

    @Override
    public void runOpMode() {

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        telemetry.addLine("Limelight Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // PURPLE DETECTION (Pipeline 0)
            // =========================
            limelight.pipelineSwitch(0);
            sleep(300); // REQUIRED delay

            LLResult purpleResult = limelight.getLatestResult();

            if (purpleResult != null && purpleResult.isValid() && purpleResult.getTa() > 0.05) {
                purpleDetected = true;
            } else {
                purpleDetected = false;
            }

            // =========================
            // GREEN DETECTION (Pipeline 3)
            // =========================
            limelight.pipelineSwitch(3);
            sleep(300); // REQUIRED delay

            LLResult greenResult = limelight.getLatestResult();

            if (greenResult != null && greenResult.isValid() && greenResult.getTa() > 0.05) {
                greenDetected = true;
            } else {
                greenDetected = false;
            }

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addData("Purple Detected", purpleDetected);
            telemetry.addData("Green Detected", greenDetected);
            telemetry.update();
        }
    }
}
