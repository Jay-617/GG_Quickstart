package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp(name = "A SnappyTeleOp PID Spinner + X Sequence FIXED (Field-Centric Merge)")
public class TeleopPIDSnappy extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    DcMotor LF, RF, LB, RB;
    DcMotor intake, outtakeL, outtakeR, spinner;
    Servo lifter, closer;
    IMU imu;
    DigitalChannel limitSwitch;

    // ---------------- PID ----------------
    double kP = 0.006, kI = 0, kD = 0.000003;
    double integral = 0, lastError = 0, integralMax = 3000;
    boolean pidEnabled = true;
    int targetTicks = 0;
    long lastTime;

    // ---------------- STALL ----------------
    int lastSpinnerPos = 0;
    boolean spinnerStalled = false;
    ElapsedTime stallTimer = new ElapsedTime();
    ElapsedTime stallCooldown = new ElapsedTime();

    // ---------------- CONSTANTS ----------------
    final double TICKS_PER_REV = 753.2;
    int ticks40, ticks55;

    // ---------------- LIMIT SWITCH ----------------
    enum LimitState { IDLE, STOP, CLEAR }
    LimitState limitState = LimitState.IDLE;
    ElapsedTime limitTimer = new ElapsedTime();

    // ---------------- X SEQUENCE ----------------
    enum XState { IDLE, BACK, LIFT, FORWARD }
    XState xState = XState.IDLE;
    ElapsedTime xTimer = new ElapsedTime();

    // ---------------- BUTTON MEMORY ----------------
    boolean lastX, lastY;
    boolean lastIntake, intakeOn;
    boolean lastOuttake, outtakeOn;
    boolean lastCloser, closerOn;
    boolean lastLifter, lifterOn;
    boolean lastInvert = false;
    boolean invertedControls = false;
    ElapsedTime buttonDelay = new ElapsedTime();
    boolean lastYawReset = false;

    @Override
    public void runOpMode() {

        // ---------------- MAP ----------------
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        intake = hardwareMap.dcMotor.get("intake");
        outtakeL = hardwareMap.dcMotor.get("outtakeL");
        outtakeR = hardwareMap.dcMotor.get("outtakeR");
        spinner = hardwareMap.dcMotor.get("spinner");

        lifter = hardwareMap.servo.get("lifter");
        closer = hardwareMap.servo.get("closer");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        spinner.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticks40 = (int)((40.0 / 360.0) * TICKS_PER_REV);
        ticks55 = (int)((55.0 / 360.0) * TICKS_PER_REV);

        lastTime = System.nanoTime();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------- DRIVE / FIELD-CENTRIC ----------------
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
                invertedControls = !invertedControls;
                buttonDelay.reset();
            }
            lastInvert = gamepad1.b;
            if (invertedControls) { lx = -lx; ly = -ly; }

            if (gamepad1.y && !lastYawReset) imu.resetYaw();
            lastYawReset = gamepad1.y;

            double yaw = getYaw();
            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);

            double tempX = lx * cosA - ly * sinA;
            double tempY = lx * sinA + ly * cosA;

            double fl = tempY + tempX + rx;
            double bl = tempY - tempX + rx;
            double fr = tempY - tempX - rx;
            double br = tempY + tempX - rx;

            double speed = 0.8 +(0.2*gamepad1.left_trigger) - (0.6 * gamepad1.right_trigger);
            setDrive(fl * speed, bl * speed, fr * speed, br * speed);

            // ---------------- LIMIT SWITCH ----------------
            boolean pressed = !limitSwitch.getState();
            switch (limitState) {
                case IDLE:
                    if (pressed) {
                        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        targetTicks = 0;
                        integral = 0;
                        lastError = 0;
                        limitTimer.reset();
                        limitState = LimitState.STOP;
                    }
                    break;
                case STOP:
                    if (limitTimer.seconds() > 0.25) {
                        targetTicks = (int)((8.0 / 360.0) * TICKS_PER_REV);
                        limitState = LimitState.CLEAR;
                    }
                    break;
                case CLEAR:
                    if (Math.abs(spinner.getCurrentPosition() - targetTicks) < 5) {
                        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        targetTicks = 0;
                        limitState = LimitState.IDLE;
                    }
                    break;
            }

            // ---------------- X BUTTON ----------------
            if (gamepad2.x && !lastX && xState == XState.IDLE) {
                xState = XState.BACK;
            }
            lastX = gamepad2.x;

            switch (xState) {
                case BACK:
                    targetTicks -= ticks40;
                    xTimer.reset();
                    xState = XState.LIFT;
                    break;
                case LIFT:
                    lifter.setPosition(0.81); // lifter up
                    xTimer.reset();
                    xState = XState.FORWARD;
                    break;
                case FORWARD:
                    if (xTimer.seconds() >= 0.5) {
                        targetTicks += ticks40;
                        lifter.setPosition(0.65); // lifter down
                        xState = XState.IDLE;
                    }
                    break;
                default:
                    break;
            }

            // ---------------- Y BUTTON ----------------
            if (gamepad2.y && !lastY) targetTicks += ticks55;
            lastY = gamepad2.y;

            // ---------------- PID ----------------
            if (pidEnabled && !spinnerStalled) {
                runPID(targetTicks);
            } else {
                spinner.setPower(0);
            }

            // ---------------- STALL DETECTION ----------------
            int pos = spinner.getCurrentPosition();
            double power = spinner.getPower();
            if (Math.abs(pos - lastSpinnerPos) < 2 && Math.abs(power) > 0.25) {
                if (stallTimer.seconds() > 0.4) {
                    spinnerStalled = true;
                    pidEnabled = false;
                    spinner.setPower(0);
                    stallCooldown.reset();
                }
            } else {
                stallTimer.reset();
            }
            lastSpinnerPos = pos;

            if (spinnerStalled && stallCooldown.seconds() > 1.0) {
                spinnerStalled = false;
                pidEnabled = true;
                integral = 0;
                lastError = 0;
            }

            // ---------------- MECHANISMS ----------------
            // Intake toggle and reverse
            if (gamepad2.a && !lastIntake) intakeOn = !intakeOn;
            lastIntake = gamepad2.a;

            double intakePower = 0;
            if (gamepad2.left_trigger > 0.1) { // reverse override
                intakePower = -0.5;
            } else if (intakeOn) {
                intakePower = 0.5;
            } else {
                intakePower = 0;
            }
            intake.setPower(intakePower);

            // Outtake
            if (gamepad2.b && !lastOuttake) outtakeOn = !outtakeOn;
            lastOuttake = gamepad2.b;
            outtakeL.setPower(outtakeOn ? -1 : 0);
            outtakeR.setPower(outtakeOn ? 1 : 0);

            // Closer
            if (gamepad2.dpad_down && !lastCloser) closerOn = !closerOn;
            lastCloser = gamepad2.dpad_down;
            closer.setPosition(closerOn ? 0.6 : 0.48);

            // Lifter
            if (xState == XState.IDLE) {
                if (gamepad2.dpad_up && !lastLifter) lifterOn = !lifterOn;
                lastLifter = gamepad2.dpad_up;
                lifter.setPosition(lifterOn ? 0.82 : 0.65);
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData("X State", xState);
            telemetry.addData("Spinner Pos", pos);
            telemetry.addData("STALL", spinnerStalled);
            telemetry.addData("Yaw", Math.toDegrees(yaw));

            telemetry.addData("Intake", intakePower > 0 ? "\u2705 ON" : (intakePower < 0 ? "\u26AB REVERSE" : "\u274C OFF"));
            telemetry.addData("Outtake", outtakeOn ? "\u2705 ON" : "\u274C OFF");
            telemetry.addData("Lifter", lifterOn ? "\u2705 UP" : "\u274C DOWN");
            telemetry.addData("Closer", closerOn ? "\u2705 UP" : "\u274C DOWN");

            telemetry.update();
        }
    }

    private void runPID(int target) {
        long now = System.nanoTime();
        double dt = Math.max(0.001, Math.min((now - lastTime) / 1e9, 0.05));
        lastTime = now;

        double error = target - spinner.getCurrentPosition();

        integral += error * dt;
        integral = Math.max(-integralMax, Math.min(integral, integralMax));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        output = Math.max(-0.3, Math.min(output, 0.3));

        spinner.setPower(output);
    }

    private double getYaw() {
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        return a.getYaw(AngleUnit.RADIANS);
    }

    private void setDrive(double fl, double bl, double fr, double br) {
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br)));
        if (max > 1.0) { fl/=max; bl/=max; fr/=max; br/=max; }

        LF.setPower(fl);
        LB.setPower(bl);
        RF.setPower(fr);
        RB.setPower(br);
    }
}
