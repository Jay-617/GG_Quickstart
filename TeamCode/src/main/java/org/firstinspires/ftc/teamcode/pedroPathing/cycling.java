package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "current cycling code")
public class cycling extends OpMode {

    private Limelight3A limelight3A;
    private DcMotor spinner;
    private DigitalChannel limitSwitch;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime limitTimer = new ElapsedTime();
    private ElapsedTime dwellTimer = new ElapsedTime();
    private ElapsedTime phaseWaitTimer = new ElapsedTime();  // For breaks between phases

    // ---- Spinner constants ----
    static final double TICKS_PER_REV = 753.2;
    static final double GEAR_RATIO = 1.0;

    // ---- Detection trigger ----
    static final double MIN_VALID_AREA = 2.0;

    // ---- Overrun to guarantee hitting limit during initial homing ----
    static final double INITIAL_OVERRUN_DEGREES = 720.0;

    // ---- Move amount for each color ----
    static final double MOVE_DEGREES = 55.0;

    // ---- Slower movement ----
    static final double MAX_PID_POWER = 0.15;

    // ---- Dwell time at each position for detection ----
    static final double DWELL_TIME_SECONDS = 1.0;  // Time to check at each stop

    // ---- Phase waits ----
    static final double AFTER_PURPLE_WAIT_SECONDS = 3.0;  // After each purple
    static final double BETWEEN_PHASE_WAIT_SECONDS = 2.0;  // After green

    // ---- PID constants ----
    private double kP = 0.006;
    private double kI = 0.0;
    private double kD = 0.000003;
    private double integral = 0;
    private double lastError = 0;
    private double integralMax = 3000;
    private long lastPidTime;

    // ---- Pipelines for colors ----
    private static final int PURPLE_PIPELINE = 1;
    private static final int GREEN_PIPELINE = 2;

    // ---- Sequence of expected colors (pipelines) ----

    // The three difference color patterns on obelisk
    private int[][] colorSequence = {
        {GREEN_PIPELINE, PURPLE_PIPELINE, PURPLE_PIPELINE}, //21
        {PURPLE_PIPELINE, GREEN_PIPELINE, PURPLE_PIPELINE}, //22
        {PURPLE_PIPELINE, PURPLE_PIPELINE, GREEN_PIPELINE}  //23
    };

    //tracks the current ID we're referencing for the pattern
    int currentid = 0;
    private int currentSequenceIndex = 0;

    // ---- State tracking ----
    private int targetTicks = 0;
    private boolean hasInitialHomed = false;
    private boolean positionJustReached = false;
    private boolean inPhaseWait = false;  // Waiting between phases
    private double currentPhaseWaitTime = 0.0;  // Dynamic wait time per phase

    // ---- Settled threshold ----
    static final int SETTLED_TICK_THRESHOLD = 10;

    // ---- Limit switch states ----
    private boolean limitActive = false;
    private boolean forwardClearStage = false;

    int numberofswitches = 0;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight3A.pipelineSwitch(colorSequence[currentid][currentSequenceIndex]);

        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        lastPidTime = System.nanoTime();
        dwellTimer.reset();
        phaseWaitTimer.reset();

        telemetry.addLine("✅ Ready - Search all colors with 55° steps + phase waits");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight3A.start();
        runtime.reset();
    }

    @Override
    public void loop() {

        // ---------- LIMIT SWITCH HOMING (highest priority) ----------
        boolean pressed = !limitSwitch.getState();
        if (pressed && !limitActive) {
            limitActive = true;
            forwardClearStage = false;
            limitTimer.reset();
            spinner.setPower(0);
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            targetTicks = 0;
            integral = 0;
            lastError = 0;

            if (!hasInitialHomed) {
                hasInitialHomed = true;
            }
        }

        //if the limit is active, then it waits three seconds
        if (limitActive) {
            if (limitTimer.seconds() < 3.0) {
                spinner.setPower(0);
            } else if (!forwardClearStage) {
                int eightDegreeTicks = (int) ((8.0 / 360.0) * TICKS_PER_REV * GEAR_RATIO);
                spinner.setTargetPosition(eightDegreeTicks);
                spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spinner.setPower(0.2);
                if (!spinner.isBusy()) {
                    spinner.setPower(0);
                    spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    forwardClearStage = true;
                    limitActive = false;
                }
            }
            runPID(targetTicks);
            telemetry.addLine("HOMING TO LIMIT SWITCH...");
            telemetry.update();
            return;
        }

        // ----------main limelight function logic ----------
        LLResult llResult = limelight3A.getLatestResult();
        double ta = 0.0;
        boolean seesExpected = false;

        if (llResult != null && llResult.isValid()) {
            ta = llResult.getTa();
            if (ta >= MIN_VALID_AREA) {
                seesExpected = true;
            }
            //String colorResults = String.join(" ", (CharSequence) llResult.getColorResults());
            //telemetry.addData("Color", "%s", colorResults);
            telemetry.addData("Target Area %", "%.2f", ta);
        } else {
            telemetry.addData("Limelight", "No result");
        }

        boolean isSettled = Math.abs(spinner.getCurrentPosition() - targetTicks) < SETTLED_TICK_THRESHOLD;

        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Pos", spinner.getCurrentPosition());
        telemetry.addData("Initial Homed", hasInitialHomed);
        telemetry.addData("Sequence Index", currentSequenceIndex);
        telemetry.addData("Sees Expected", seesExpected);
        telemetry.addData("Dwell Time", dwellTimer.seconds());
        telemetry.addData("Phase Wait Time", phaseWaitTimer.seconds());
        telemetry.addData("In Phase Wait", inPhaseWait);
        telemetry.addData("# switches",numberofswitches);

        // ---------- PHASE 0: Initial overrun to hit limit and home ----------
        if (!hasInitialHomed) {
            addDegreesToTarget(INITIAL_OVERRUN_DEGREES);
        }

        // ---------- Color Sequence Processing ----------
        //we're finding the current sequence and id and making sure it's the proper length, not going past 3 colors
        else if (currentSequenceIndex < colorSequence[currentid].length) {

            if (inPhaseWait) {
                // Hold during inter-phase wait
                if (phaseWaitTimer.seconds() >= currentPhaseWaitTime) {
                    inPhaseWait = false;
                    if (currentSequenceIndex < colorSequence[currentid].length) {
                        limelight3A.pipelineSwitch(colorSequence[currentid][currentSequenceIndex]);
                        numberofswitches += 1;

                        addDegreesToTarget(MOVE_DEGREES);  // Start search with first move
                        positionJustReached = false;
                    }
                }
            } else {
                // Detect settle
                if (isSettled && !positionJustReached) {
                    positionJustReached = true;
                    dwellTimer.reset();
                } else if (!isSettled) {
                    positionJustReached = false;
                }

                if (isSettled && positionJustReached && dwellTimer.seconds() >= DWELL_TIME_SECONDS) {
                    if (seesExpected) {
                        // Found: final 55° move for this color
                        addDegreesToTarget(MOVE_DEGREES);
                        positionJustReached = false;

                        // Prepare next phase with wait
                        currentSequenceIndex++;
                        if (currentSequenceIndex < colorSequence[currentid].length) {
                            inPhaseWait = true;
                            phaseWaitTimer.reset();
                            currentPhaseWaitTime = (colorSequence[currentid][currentSequenceIndex - 1] == PURPLE_PIPELINE)
                                    ? AFTER_PURPLE_WAIT_SECONDS : BETWEEN_PHASE_WAIT_SECONDS;
                        }
                    } else {
                        // Not found: search next slot
                        addDegreesToTarget(MOVE_DEGREES);
                        positionJustReached = false;
                    }
                }
            }
        }

        // ---------- After sequence: hold position (stop completely) ----------
        // No code here → PID holds last targetTicks

        // ---------- Always run PID ----------
        runPID(targetTicks);

        telemetry.update();
    }

    private void addDegreesToTarget(double degrees) {
        int ticks = (int) ((degrees / 360.0) * TICKS_PER_REV * GEAR_RATIO);
        targetTicks += ticks;
    }

    private void runPID(int target) {
        long now = System.nanoTime();
        double dt = (now - lastPidTime) / 1e9;
        dt = Math.max(0.001, Math.min(dt, 0.05));
        lastPidTime = now;

        double pos = spinner.getCurrentPosition();
        double error = target - pos;

        integral += error * dt;
        integral = Math.max(-integralMax, Math.min(integral, integralMax));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        output = Math.max(-MAX_PID_POWER, Math.min(output, MAX_PID_POWER));

        spinner.setPower(output);
    }
}