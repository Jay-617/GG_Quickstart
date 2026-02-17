package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.List;

@Autonomous(name = "Two April Tags", group = "Pedro")
public class limelightapriltag extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private Pose3D tag1Pose = null;
    private Pose3D tag2Pose = null;

    private int tag1ID = -1;
    private int tag2ID = -1;

    private double tag1Tx = 0;
    private double tag1Ty = 0;
    private double tag1Ta = 0;

    private double tag2Tx = 0;
    private double tag2Ty = 0;
    private double tag2Ta = 0;

    // pipeline control
    private int currentPipeline = 1; // start with pipeline 1
    private long pipelineSwitchTime = 0;
    private static final long PIPELINE_SETTLE_MS = 250;

    @Override
    public void init() {
        // start with pipeline 1 (first tag)
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(1);

        // direction
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        pipelineSwitchTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        // Update robot heading in radians
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(Math.toRadians(angles.getYaw()));

        LLResult result = limelight.getLatestResult();

        // wait for pipeline to settle before reading
        if (System.currentTimeMillis() - pipelineSwitchTime < PIPELINE_SETTLE_MS) {
            telemetry.addLine("Waiting for pipeline to settle...");
            telemetry.update();
            return;
        }

        // if the result
        if (result != null && result.isValid()) {

            // captures april tag ID and position to create a list
            List<FiducialResult> fiducials = result.getFiducialResults();
            telemetry.addData("Fiducials Detected", fiducials.size());

            for (FiducialResult fr : fiducials) {
                int detectedID = fr.getFiducialId();
                telemetry.addData("Current ID", detectedID);

                // FIRST TAG (Pipeline 1)
                if (currentPipeline == 1 && tag1ID == -1) {
                    tag1ID = detectedID;
                    tag1Pose = result.getBotpose();

                    // use FiducialResult values
                    tag1Tx = fr.getTargetXDegrees();
                    tag1Ty = fr.getTargetYDegrees();
                    tag1Ta = fr.getTargetArea();

                    // switch to pipeline 0 for second tag
                    limelight.pipelineSwitch(0);
                    currentPipeline = 0;
                    pipelineSwitchTime = System.currentTimeMillis();
                    break;
                }

                // SECOND TAG (Pipeline 0)
                else if (currentPipeline == 0 && tag2ID == -1 && detectedID != tag1ID) {
                    tag2ID = detectedID;
                    tag2Pose = result.getBotpose();

                    tag2Tx = fr.getTargetXDegrees();
                    tag2Ty = fr.getTargetYDegrees();
                    tag2Ta = fr.getTargetArea();
                    break;
                }
            }
        }

        telemetry.addLine("");

        // show ID or Not Detected
        telemetry.addData("Saved 1 ID", tag1ID == -1 ? "Not Detected" : tag1ID);
        telemetry.addData("Saved 2 ID", tag2ID == -1 ? "Not Detected" : tag2ID);

        telemetry.addLine("Pose values");

        //these variables are for the obelisk
        telemetry.addData("Tag 1 Tx", tag1Tx);
        telemetry.addData("Tag 1 Ty", tag1Ty);
        telemetry.addData("Tag 1 Ta", tag1Ta);

        //these variables for the goal
        telemetry.addData("Tag 2 Tx", tag2Tx);
        telemetry.addData("Tag 2 Ty", tag2Ty);
        telemetry.addData("Tag 2 Ta", tag2Ta);

        // Stop ONLY after both unique tags are captured
        if (tag1Pose != null && tag2Pose != null) {
            telemetry.addLine("BOTH APRILTAGS CAPTURED");
            // requestOpModeStop();
        }

        //to add
        // if detected ID is 21, use green, purple, purple
        // if detected ID is 22, use purple, green, purple
        // if detected ID is 23, use purple, purple, green
        // if tag detected is #, use ENUM, and set that motif to a certain key

        telemetry.update();
    }
}
