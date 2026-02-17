package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

@Autonomous(name = "Spinner Pattern for ID 23 PPG", group = "Vision")
public class DetectArtifactGreen extends LinearOpMode {

    private DcMotor spinner;
    private ColorBlobLocatorProcessor greenLocator;
    private ColorBlobLocatorProcessor purpleLocator;
    private VisionPortal portal;

    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
         * FIXED & MATCHED TO SAMPLE
         * Use Unity coordinates for ROI
         * Disable unnecessary overlays (required in new SDK)
         * Keep blur size at 5 (optimal for 320x240)
         */

        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(0, 255, 0))
                .setBlurSize(5)
                .build();

        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 0, 255))
                .setBlurSize(5)
                .build();

        telemetry.addLine("Initializing camera...");
        telemetry.update();



        /*
         * FIXED: VisionPortal must be built DURING INIT
         * This guarantees the stream loads correctly.
         */
        portal = new VisionPortal.Builder()
                .addProcessor(greenLocator)
                .addProcessor(purpleLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.addLine("Camera Initialized!");
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        // Keep INIT alive so the preview stream initializes
        while (opModeInInit()) {
            telemetry.addData("Green blobs", greenLocator.getBlobs().size());
            telemetry.addData("Purple blobs", purpleLocator.getBlobs().size());
            telemetry.update();
        }

        // --- Start pressed ---
        int targetID = 23;
        String[] pattern = {"PURPLE", "PURPLE", "GREEN"};

        for (String needed : pattern) {
            boolean matched = false;

            while (opModeIsActive() && !matched) {

                boolean greenSeen = greenLocator.getBlobs().size() > 0;
                boolean purpleSeen = purpleLocator.getBlobs().size() > 0;

                String detected = "NONE";
                if (greenSeen) detected = "GREEN";
                else if (purpleSeen) detected = "PURPLE";

                telemetry.addData("Target Color", needed);
                telemetry.addData("Detected", detected);
                telemetry.update();

                if ((needed.equals("GREEN") && greenSeen) ||
                        (needed.equals("PURPLE") && purpleSeen)) {

                    spinner.setPower(0.5);
                    sleep(1000);
                    spinner.setPower(0);

                    matched = true;
                } else {
                    spinner.setPower(0);
                }

                sleep(50);
            }
        }

        spinner.setPower(0);
        telemetry.addLine("Sequence complete!");
        telemetry.update();
    }
}

