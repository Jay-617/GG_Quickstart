//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name="Mecanum Drive To AprilTag ID20", group="Auto")
//public class MecanumDriveToAprilTag extends LinearOpMode {
//
//    private DcMotor LF, RF, LB, RB;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    private static final boolean USE_WEBCAM = true;
//    private static final int DESIRED_TAG_ID = 20; // now only track ID 20
//    private static final double DESIRED_DISTANCE = 12.0;
//
//    private static final double SPEED_GAIN = 0.02;
//    private static final double STRAFE_GAIN = 0.04;
//    private static final double TURN_GAIN  = 0.01;
//
//    private static final double MAX_AUTO_SPEED  = 0.5;
//    private static final double MAX_AUTO_STRAFE = 0.5;
//    private static final double MAX_AUTO_TURN   = 0.25;
//
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection storedTag = null; // store last seen tag
//
//    @Override
//    public void runOpMode() {
//
//        // --- Hardware mapping ---
//        LF = hardwareMap.get(DcMotor.class, "FL");
//        LB = hardwareMap.get(DcMotor.class, "BL");
//        RF = hardwareMap.get(DcMotor.class, "FR");
//        RB = hardwareMap.get(DcMotor.class, "BR");
//
//        // Set directions
//        LF.setDirection(DcMotor.Direction.REVERSE);
//        LB.setDirection(DcMotor.Direction.REVERSE);
//        RF.setDirection(DcMotor.Direction.FORWARD);
//        RB.setDirection(DcMotor.Direction.FORWARD);
//
//        // Initialize AprilTag
//        initAprilTag();
//        if (USE_WEBCAM) setManualExposure(6, 250);
//
//        telemetry.addLine("Ready");
//        telemetry.update();
//        waitForStart();
//        runtime.reset();
//
//        while (opModeIsActive()) {
//
//            // --- Get detections ---
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//            telemetry.addLine("Detected IDs:");
//            for (AprilTagDetection detection : currentDetections) {
//                telemetry.addData("ID", detection.id);
//                // store tag if it's the desired ID
//                if (detection.id == DESIRED_TAG_ID && detection.ftcPose != null) {
//                    storedTag = detection;
//                }
//            }
//
//            double drive = 0;
//            double strafe = 0;
//            double turn = 0;
//
//            if (storedTag != null && storedTag.ftcPose != null) {
//                double range = storedTag.ftcPose.range;
//                double bearing = storedTag.ftcPose.bearing;
//                double xOffset = storedTag.ftcPose.x;
//
//                double rangeError = range - DESIRED_DISTANCE;
//                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                strafe = Range.clip(xOffset * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//                turn = Range.clip(bearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//
//                // Stop if within 1 inch
//                if (Math.abs(rangeError) < 1.0) {
//                    drive = 0; strafe = 0; turn = 0;
//                    telemetry.addLine("Reached target coordinates!");
//                }
//
//                telemetry.addData("Driving to stored ID", DESIRED_TAG_ID);
//                telemetry.addData("Range", "%.2f in", range);
//                telemetry.addData("Bearing", "%.2f deg", bearing);
//                telemetry.addData("X Offset", "%.2f in", xOffset);
//
//            } else {
//                telemetry.addLine("Searching for ID 20...");
//            }
//
//            // Apply mecanum drive math
//            double fl = drive + strafe + turn;
//            double fr = drive - strafe - turn;
//            double bl = drive - strafe + turn;
//            double br = drive + strafe - turn;
//
//            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
//            if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }
//
//            LF.setPower(fl);
//            RF.setPower(fr);
//            LB.setPower(bl);
//            RB.setPower(br);
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        // Stop motors at end
//        LF.setPower(0); RF.setPower(0); LB.setPower(0); RB.setPower(0);
//        telemetry.addLine("Autonomous finished");
//        telemetry.update();
//        sleep(500);
//    }
//
//    private void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(aprilTag)
//                    .build();
//        }
//    }
//
//    private void setManualExposure(int exposureMS, int gain) {
//        if (visionPortal == null) return;
//        while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) { sleep(20); }
//        if (!isStopRequested()) {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual); sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS); sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain); sleep(20);
//        }
//    }
//}
