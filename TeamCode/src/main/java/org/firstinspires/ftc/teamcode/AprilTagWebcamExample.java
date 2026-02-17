//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//@Autonomous
//public class AprilTagWebcamExample extends OpMode {
//    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
//
//    @Override
//    public void init() {
//        aprilTagWebcam.init(hardwareMap, telemetry);
//    }
//
//    @Override
//    public void loop() {
//        // Update the vision portal (it will only detect and save once)
//        aprilTagWebcam.update();
//
//        // Get the saved ID
//        int savedId = aprilTagWebcam.getSavedTagId();
//        if (savedId != -1) {
//            // Optionally, get the detection details if needed (using the last detected list)
//            AprilTagDetection detection = aprilTagWebcam.getTagBySpecificId(savedId);
//            aprilTagWebcam.displayDetectionTelemetry(detection);
//            // You can use savedId for other logic here, e.g., decision making in autonomous
//        } else {
//            telemetry.addLine("No tag detected yet.");
//        }
//    }
//}