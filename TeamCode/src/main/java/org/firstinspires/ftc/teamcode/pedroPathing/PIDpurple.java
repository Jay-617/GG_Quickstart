package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Limelight Purple Cycle - Home First + Wait + 55° on Detect")
public class PIDpurple extends OpMode {

    private Limelight3A limelight3A;
    private DcMotor spinner;
    private DigitalChannel limitSwitch;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime limitTimer = new ElapsedTime();

    // ---- Spinner constants ----
    static final double TICKS_PER_REV = 753.2;
    static final double GEAR_RATIO = 1.0;

    // ---- Detection trigger (any visible purple) ----
    static final double MIN_VALID_AREA = 2.0;  // Just needs to see some purple to trigger

    // ---- Overrun to guarantee hitting limit during initial homing ----
    static final double INITIAL_OVERRUN_DEGREES = 720.0;  // 2 full revolutions - adjust if needed to ensure limit hit

    // ---- Final move when purple detected ----
    static final double FINAL_MOVE_DEGREES = 55.0;

    // ---- PID constants (from your teleop) ----
    private double kP = 0.006;
    private double kI = 0.0;
    private double kD = 0.000003;
    private double integral = 0;
    private double lastError = 0;
    private double integralMax = 3000;
    private long lastPidTime;

    // ---- State tracking ----
    private int targetTicks = 0;
    private boolean hasInitialHomed = false;
    private boolean hasDetectedPurple = false;
    private boolean hasMoved55 = false;

    // ---- Limit switch states ----
    private boolean limitActive = false;
    private boolean forwardClearStage = false;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight3A.pipelineSwitch(1);  // Purple pipeline

        spinner = hardwareMap.dcMotor.get("spinner");
        spinner.setDirection(DcMotor.Direction.REVERSE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        lastPidTime = System.nanoTime();

        telemetry.addLine("✅ Ready - Will home first, then wait for purple");
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

            // If this is the initial homing, mark it complete
            if (!hasInitialHomed) {
                hasInitialHomed = true;
            }
        }

        if (limitActive) {
            if (limitTimer.seconds() < 0.5) {
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

        // ---------- VISION ----------
        LLResult llResult = limelight3A.getLatestResult();
        double ta = 0.0;
        boolean seesPurple = false;

        if (llResult != null && llResult.isValid()) {
            ta = llResult.getTa();
            if (ta >= MIN_VALID_AREA) {
                seesPurple = true;
            }
            telemetry.addData("Target Area %", "%.2f", ta);
        } else {
            telemetry.addData("Limelight", "No result");
        }

        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Pos", spinner.getCurrentPosition());
        telemetry.addData("Initial Homed", hasInitialHomed);
        telemetry.addData("Sees Purple", seesPurple);
        telemetry.addData("Moved 55°", hasMoved55);

        // ---------- PHASE 0: Initial overrun to hit limit and home ----------
        if (!hasInitialHomed) {
            addDegreesToTarget(INITIAL_OVERRUN_DEGREES);  // Keep adding until limit triggers homing
        }

        // ---------- PHASE 1: Wait at home position until purple detected ----------
        else if (!hasDetectedPurple) {
            targetTicks = 0;  // Hold at zero
            if (seesPurple) {
                hasDetectedPurple = true;
            }
        }

        // ---------- PHASE 2: When purple first seen → move exactly 55° ----------
        else if (!hasMoved55) {
            addDegreesToTarget(FINAL_MOVE_DEGREES);
            hasMoved55 = true;
        }

        // ---------- After 55° move: just hold position (no further movement) ----------
        // Nothing to do here - PID will hold the 55° position forever

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
        output = Math.max(-0.3, Math.min(output, 0.3));

        spinner.setPower(output);
    }
}