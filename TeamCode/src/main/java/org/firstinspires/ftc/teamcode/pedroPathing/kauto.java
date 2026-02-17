//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.AprilTagWebcam;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//@Autonomous(name = "kauto", group = "Main")
//public class kauto extends LinearOpMode {
//    // --- Drive motors ---
//    private DcMotor frontLeft, backLeft, frontRight, backRight;
//
//    // --- Vision ---
//    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
//
//    // --- Target distance for tag (cm) ---
//    private static final double TARGET_DISTANCE_CM = 205.74;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // --- Initialize motors ---
//        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
//        backLeft   = hardwareMap.get(DcMotor.class, "BL");
//        frontRight = hardwareMap.get(DcMotor.class, "FR");
//        backRight  = hardwareMap.get(DcMotor.class, "BR");
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        // --- Initialize camera ---
//        aprilTagWebcam.init(hardwareMap, telemetry);
//
//        telemetry.addLine("Initialization complete — looking for tag...");
//        telemetry.update();
//
//        // --- Wait for start and detect tag during init ---
//        AprilTagDetection tag = null;
//        while (opModeInInit() && !isStopRequested()) {
//            aprilTagWebcam.update();
//
//            // Debug: show all detected IDs
//            for (AprilTagDetection d : aprilTagWebcam.getDetectedTags()) {
//                telemetry.addData("Detected ID", d.id);
//            }
//
//            // Check your specific tag
//            tag = aprilTagWebcam.getTagBySpecificId(20);
//            if (tag != null) {
//                aprilTagWebcam.displayDetectionTelemetry(tag);
//            } else {
//                telemetry.addLine("No tag detected yet...");
//            }
//            telemetry.update();
//        }
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // --- Autonomous movement until target distance ---
//        AprilTagDetection lastTag = tag; // start with tag detected during init, if any
//
//        while (opModeIsActive()) {
//            aprilTagWebcam.update();
//
//            // Debug: show all detected IDs
//            for (AprilTagDetection d : aprilTagWebcam.getDetectedTags()) {
//                telemetry.addData("Detected ID", d.id);
//            }
//
//            // Check for the specific tag
//            tag = aprilTagWebcam.getTagBySpecificId(20);
//            if (tag != null) {
//                lastTag = tag; // update last seen tag
//                aprilTagWebcam.displayDetectionTelemetry(tag);
//            }
//
//            if (lastTag != null) {
//                double currentDistance = lastTag.ftcPose.range; // cm
//                telemetry.addData("Current Distance (cm)", "%.2f", currentDistance);
//                telemetry.update();
//
//                // Move robot toward target distance
//                driveToDistance(TARGET_DISTANCE_CM, currentDistance, 0.3);
//            } else {
//                // Haven’t seen tag yet → stop
//                setLeftDrivePower(0);
//                setRightDrivePower(0);
//                telemetry.addLine("Tag not yet detected — waiting...");
//                telemetry.update();
//            }
//        }
//
//        // --- Stop all motors and camera ---
//        setLeftDrivePower(0);
//        setRightDrivePower(0);
//        aprilTagWebcam.stop();
//        telemetry.addLine("Autonomous complete");
//        telemetry.update();
//    }
//
//    // =====================
//    // --- Helper Methods ---
//    // =====================
//
//    // Set both left motors
//    private void setLeftDrivePower(double power) {
//        frontLeft.setPower(power);
//        backLeft.setPower(power);
//    }
//
//    // Set both right motors
//    private void setRightDrivePower(double power) {
//        frontRight.setPower(power);
//        backRight.setPower(power);
//    }
//
//    /**
//     * Moves robot forward/backward based on current distance to tag.
//     *
//     * @param targetDistanceCm desired distance from the tag in cm
//     * @param currentDistanceCm current distance from the tag in cm
//     * @param maxPower maximum motor power to use (0-1)
//     */
//    private void driveToDistance(double targetDistanceCm, double currentDistanceCm, double maxPower) {
//        double tolerance = 2.0; // cm
//
//        if (currentDistanceCm > targetDistanceCm + tolerance) {
//            // Too far → move backward
//            setLeftDrivePower(-maxPower);
//            setRightDrivePower(-maxPower);
//        } else if (currentDistanceCm < targetDistanceCm - tolerance) {
//            // Too close → move forward
//            setLeftDrivePower(maxPower);
//            setRightDrivePower(maxPower);
//        } else {
//            // Within tolerance → stop
//            setLeftDrivePower(0);
//            setRightDrivePower(0);
//        }
//    }
//}
