package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Artifact Spin", group = "AUTO")
public class ArtifactScan extends LinearOpMode {

    // Mechanisms
    private DcMotor intake, outtakeL, outtakeR, spinner;
    private Servo lifter, closer;

    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera webcam;
    private ArtifactPipeline pipeline; // Pattern detection pipeline

    // Spinner constants
    static final double SPIN_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeL = hardwareMap.get(DcMotor.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotor.class, "outtakeR");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        lifter = hardwareMap.get(Servo.class, "lifter");
        closer = hardwareMap.get(Servo.class, "closer");

        // Set initial positions
        lifter.setPosition(0.65);
        closer.setPosition(0.48);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Activate both intakes
        intake.setPower(0.5);

        // --- Webcam setup ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ArtifactPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("✅ Ready to start");
        telemetry.update();
        waitForStart();

        // --- Wait for a pattern to be detected ---
        ArtifactPipeline.Pattern detectedPattern = ArtifactPipeline.Pattern.NONE;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) { // timeout after 5s
            detectedPattern = pipeline.getPattern();
            telemetry.addData("Detected Pattern", detectedPattern);
            telemetry.update();

            if (detectedPattern != ArtifactPipeline.Pattern.NONE) break;
        }

        // Stop webcam once pattern is detected or timed out
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        // --- Execute mechanism sequence based on pattern ---
        switch (detectedPattern) {
            case GREEN_PURPLE_PURPLE:
                telemetry.addLine("Pattern: GREEN_PURPLE_PURPLE");
                telemetry.update();
                executeArtifactAction(10);
                break;

            case PURPLE_GREEN_PURPLE:
                telemetry.addLine("Pattern: PURPLE_GREEN_PURPLE");
                telemetry.update();
                executeArtifactAction(20);
                break;

            case PURPLE_PURPLE_GREEN:
                telemetry.addLine("Pattern: PURPLE_PURPLE_GREEN");
                telemetry.update();
                executeArtifactAction(30);
                break;

            default:
                telemetry.addLine("No valid pattern detected. Skipping action.");
                telemetry.update();
        }

        telemetry.addLine("✅ Finished");
        telemetry.update();
        sleep(500);
    }

    private void executeArtifactAction(int spinnerDegrees) {
        if (spinner == null) return;

        // --- Move spinner ---
        int target = spinner.getCurrentPosition() + (int)(spinnerDegrees * 384.5 / (7.1 * Math.PI));
        spinner.setTargetPosition(target);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(Math.abs(SPIN_SPEED));

        while (opModeIsActive() && spinner.isBusy()) {
            telemetry.addData("Spinner Pos", spinner.getCurrentPosition());
            telemetry.update();
        }

        spinner.setPower(0);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Activate pusher (simulate with closer/lifter) ---
        closer.setPosition(0.6);
        sleep(400);
        closer.setPosition(0.48);

        // --- Activate outtake ---
        outtakeL.setPower(0.6);
        outtakeR.setPower(0.6);
        sleep(1000);
        outtakeL.setPower(0);
        outtakeR.setPower(0);
    }
}
