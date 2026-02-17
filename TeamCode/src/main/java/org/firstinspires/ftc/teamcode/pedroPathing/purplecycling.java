package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "LimelightCycling + Spinner (Accurate Offset + Auto Adjust)")
public class purplecycling extends OpMode {

    private Limelight3A limelight3A;
    private DcMotor spinner;
    private ElapsedTime runtime = new ElapsedTime();

    // ---- Spinner constants ----
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double SPINNER_POWER = 0.6;

    // ---- Detection trigger ----
    static final double TARGET_AREA_TRIGGER = 8.0;     // Desired % area
    static final double AREA_TOLERANCE = 1.5;          // Tolerance for final trigger

    // ---- Auto-adjust search parameters ----
    static final double SEARCH_DEGREES_STEP = 10.0;     // Small step size for searching (tune this)
    static final int MAX_SEARCH_STEPS = 36;            // Safety: max ~360 degrees of searching (36*10)
    static final double MIN_VALID_AREA = 2.0;          // Minimum area to consider "detected" during search

    // ---- Accurate offset ----
    static final double SPINNER_OFFSET_DEGREES = 72.0;

    private boolean hasAdjusted = false;
    private boolean hasSpun = false;
    private int searchStepCount = 0;
    private boolean searchDirectionPositive = true;    // Start searching one way; can add logic to reverse if needed

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight3A.pipelineSwitch(0);

        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hasAdjusted = false;
        hasSpun = false;
        searchStepCount = 0;

        telemetry.addLine("✅ Limelight + Spinner Ready (With Auto-Adjust Search)");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight3A.start();
        runtime.reset();
    }

    @Override
    public void loop() {

        LLResult llResult = limelight3A.getLatestResult();

        double ta = 0.0;
        boolean validDetection = false;

        if (llResult != null && llResult.isValid()) {
            ta = llResult.getTa();
            validDetection = true;

            telemetry.addData("Target Area (ta %)", "%.2f", ta);
        } else {
            telemetry.addData("Limelight", "No valid result");
        }

        telemetry.addData("Search Steps", searchStepCount);
        telemetry.addData("Has Adjusted", hasAdjusted);
        telemetry.addData("Has Spun Offset", hasSpun);

        // Phase 1: Auto-adjust by small spins until good area detected
        if (!hasAdjusted) {
            if (validDetection && Math.abs(ta - TARGET_AREA_TRIGGER) <= AREA_TOLERANCE) {
                // Already perfect – skip directly to offset
                hasAdjusted = true;
            } else if (searchStepCount < MAX_SEARCH_STEPS) {
                if (!spinner.isBusy()) {  // Previous move finished
                    if (validDetection && ta >= MIN_VALID_AREA) {
                        // Good enough detection – proceed to offset
                        hasAdjusted = true;
                    } else {
                        // Spin a small step and check again
                        spinSpinnerDegrees(searchDirectionPositive ? SEARCH_DEGREES_STEP : -SEARCH_DEGREES_STEP);
                        searchStepCount++;
                    }
                }
            } else {
                telemetry.addLine("⚠️ Search timeout – proceeding anyway");
                hasAdjusted = true;  // Give up and try offset
            }
        }

        // Phase 2: Once adjusted, apply precise offset
        if (hasAdjusted && !hasSpun) {
            if (!spinner.isBusy()) {
                spinSpinnerDegrees(SPINNER_OFFSET_DEGREES);
                hasSpun = true;
            }
        }

        telemetry.update();
    }

    // Precise spinner movement (unchanged, but now used for both search and offset)
    private void spinSpinnerDegrees(double degrees) {
        int ticks = (int) ((degrees / 360.0) * COUNTS_PER_MOTOR_REV);
        int target = spinner.getCurrentPosition() + ticks;

        spinner.setTargetPosition(target);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setPower(SPINNER_POWER);

        runtime.reset();
        while (runtime.seconds() < 3.0 && spinner.isBusy()) {
            // Wait (increased timeout slightly for safety)
        }

        spinner.setPower(0);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}