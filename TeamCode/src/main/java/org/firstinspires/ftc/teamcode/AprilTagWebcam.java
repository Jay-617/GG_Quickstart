package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;
    private AprilTagDetection savedDetection = null; // Full saved detection

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        if (savedDetection == null) {
            // Still scanning...
            telemetry.addLine("Status: Scanning for AprilTag...");
            detectedTags = aprilTagProcessor.getDetections();

            if (!detectedTags.isEmpty()) {
                // Save the first detected tag
                savedDetection = detectedTags.get(0);

                // Stop further AprilTag processing to save CPU
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);

                telemetry.addLine("✓ AprilTag DETECTED and SAVED!");
                telemetry.addLine("ID: " + savedDetection.id);
            }
        }

        // Always show the current status / saved tag info
        showSavedTagTelemetry();
    }

    // New: Dedicated telemetry method for the saved tag
    private void showSavedTagTelemetry() {
        telemetry.addLine(""); // Blank line for spacing

        if (savedDetection == null) {
            telemetry.addLine("Saved Tag: NONE");
            telemetry.addLine("Waiting for first detection...");
        } else {
            telemetry.addLine("=== SAVED APRILTAG ===");
            telemetry.addLine(String.format("ID: %d", savedDetection.id));

            if (savedDetection.metadata != null) {
                telemetry.addLine(String.format("Name: %s", savedDetection.metadata.name));
            } else {
                telemetry.addLine("Name: Unknown");
            }

            telemetry.addLine(String.format("Range:     %6.1f cm", savedDetection.ftcPose.range));
            telemetry.addLine(String.format("Bearing:   %6.1f deg", savedDetection.ftcPose.bearing));
            telemetry.addLine(String.format("Elevation: %6.1f deg", savedDetection.ftcPose.elevation));

            telemetry.addLine("Position (cm):");
            telemetry.addLine(String.format("  X: %6.1f", savedDetection.ftcPose.x));
            telemetry.addLine(String.format("  Y: %6.1f", savedDetection.ftcPose.y));
            telemetry.addLine(String.format("  Z: %6.1f", savedDetection.ftcPose.z));

            telemetry.addLine("Rotation (deg):");
            telemetry.addLine(String.format("  Pitch:  %6.1f", savedDetection.ftcPose.pitch));
            telemetry.addLine(String.format("  Roll:   %6.1f", savedDetection.ftcPose.roll));
            telemetry.addLine(String.format("  Yaw:    %6.1f", savedDetection.ftcPose.yaw));
        }

        telemetry.addLine(""); // Spacing at bottom
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getSavedDetection() {
        return savedDetection;
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        if (savedDetection != null && savedDetection.id == id) {
            return savedDetection;
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}