package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ArtifactPipeline extends OpenCvPipeline {

    // Enum for detected artifact color
    public enum ArtifactColor {
        GREEN,
        PURPLE,
        NONE
    }

    // Enum for detected pattern
    public enum Pattern {
        GREEN_PURPLE_PURPLE,
        PURPLE_GREEN_PURPLE,
        PURPLE_PURPLE_GREEN,
        NONE
    }

    // Visualization colors
    public final Scalar ARTIFACT_GREEN = new Scalar(0, 255, 0);
    public final Scalar ARTIFACT_PURPLE = new Scalar(128, 0, 128);

    // Region definitions
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 150);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150, 150);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(300, 150);
    static final int REGION_WIDTH = 45;
    static final int REGION_HEIGHT = 45;

    Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // HSV masks for green/purple detection
    private Mat hsv = new Mat();
    private Mat greenMask = new Mat();
    private Mat purpleMask = new Mat();

    // Region Mats
    private Mat region1_green, region2_green, region3_green;
    private Mat region1_purple, region2_purple, region3_purple;

    // Averages
    private int avg1_green, avg2_green, avg3_green;
    private int avg1_purple, avg2_purple, avg3_purple;

    // Detected artifact
    private volatile ArtifactColor detectedColor = ArtifactColor.NONE;

    // Detected pattern
    private volatile Pattern detectedPattern = Pattern.NONE;

    private Telemetry telemetry;

    // HSV thresholds for artifacts
    private final Scalar lowerGreen = new Scalar(40, 50, 50);
    private final Scalar upperGreen = new Scalar(80, 255, 255);

    private final Scalar lowerPurple = new Scalar(130, 50, 50);
    private final Scalar upperPurple = new Scalar(160, 255, 255);

    public ArtifactPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {
        Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, lowerGreen, upperGreen, greenMask);
        Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);

        region1_green = greenMask.submat(new Rect(region1_pointA, region1_pointB));
        region2_green = greenMask.submat(new Rect(region2_pointA, region2_pointB));
        region3_green = greenMask.submat(new Rect(region3_pointA, region3_pointB));

        region1_purple = purpleMask.submat(new Rect(region1_pointA, region1_pointB));
        region2_purple = purpleMask.submat(new Rect(region2_pointA, region2_pointB));
        region3_purple = purpleMask.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, lowerGreen, upperGreen, greenMask);
        Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);

        avg1_green = (int) Core.mean(region1_green).val[0];
        avg2_green = (int) Core.mean(region2_green).val[0];
        avg3_green = (int) Core.mean(region3_green).val[0];

        avg1_purple = (int) Core.mean(region1_purple).val[0];
        avg2_purple = (int) Core.mean(region2_purple).val[0];
        avg3_purple = (int) Core.mean(region3_purple).val[0];

        // Decide color in each region
        ArtifactColor r1 = getRegionColor(avg1_green, avg1_purple);
        ArtifactColor r2 = getRegionColor(avg2_green, avg2_purple);
        ArtifactColor r3 = getRegionColor(avg3_green, avg3_purple);

        // Determine pattern
        if (r1 == ArtifactColor.GREEN && r2 == ArtifactColor.PURPLE && r3 == ArtifactColor.PURPLE)
            detectedPattern = Pattern.GREEN_PURPLE_PURPLE;
        else if (r1 == ArtifactColor.PURPLE && r2 == ArtifactColor.GREEN && r3 == ArtifactColor.PURPLE)
            detectedPattern = Pattern.PURPLE_GREEN_PURPLE;
        else if (r1 == ArtifactColor.PURPLE && r2 == ArtifactColor.PURPLE && r3 == ArtifactColor.GREEN)
            detectedPattern = Pattern.PURPLE_PURPLE_GREEN;
        else
            detectedPattern = Pattern.NONE;

        // Draw rectangles for regions
        Imgproc.rectangle(input, region1_pointA, region1_pointB, ARTIFACT_GREEN, 2);
        Imgproc.rectangle(input, region2_pointA, region2_pointB, ARTIFACT_GREEN, 2);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, ARTIFACT_GREEN, 2);

        telemetry.addData("Detected Pattern", detectedPattern);
        telemetry.addData("Region Colors", r1 + " | " + r2 + " | " + r3);
        telemetry.update();

        return input;
    }

    private ArtifactColor getRegionColor(int greenVal, int purpleVal) {
        if (greenVal > purpleVal && greenVal > 20) return ArtifactColor.GREEN;
        else if (purpleVal > greenVal && purpleVal > 20) return ArtifactColor.PURPLE;
        else return ArtifactColor.NONE;
    }

    public Pattern getPattern() {
        return detectedPattern;
    }

    public ArtifactColor getAnalysis() {
        return detectedColor;
    }
}
