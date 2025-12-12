package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "ASNAPPYField-Centric Drive + Mechanisms + Limit Switch PID + X Spin 30° Lifter Sequence")
public class TeleopPIDSnappy extends LinearOpMode {

    // Drive motors
    private DcMotor LF, RF, LB, RB;

    // Mechanisms
    private DcMotor intake, outtakeL, outtakeR, spinner;
    private Servo lifter, closer;

    // IMU
    private IMU imu;

    // Limit switch
    private DigitalChannel limitSwitch;
    private boolean limitActive = false;
    private boolean forwardClearStage = false;
    private ElapsedTime limitTimer = new ElapsedTime();

    // Button states
    private boolean lastIntakeButton = false, intakeOn = false;
    private boolean lastOuttakeButton = false, outtakeOn = false;
    private boolean lastLifterButton = false, lifterOn = false;
    private boolean lastCloserButton = false, closerOn = false;
    private boolean lastInvert = false, lastYawReset = false, lastY = false;
    private boolean lastX = false;
    private boolean invertedControls = false;
    private ElapsedTime buttonDelay = new ElapsedTime();

    // PID constants
    double kP = 0.006, kI = 0, kD = 0.000003;
    double integral = 0, lastError = 0, integralMax = 3000;

    // Spinner
    private int targetTicks = 0;
    double TICKS_PER_REV = 753.2, GEAR_RATIO = 1.0;
    private int ticksPer55Degrees;
    private int ticksPer30Degrees;
    private long lastTime;

    // ---------------- X Button Non-blocking sequence ----------------
    private enum XSequence { IDLE, SPIN_30, LIFTER_UP, WAIT, LIFTER_DOWN }
    private XSequence xSeqState = XSequence.IDLE;
    private ElapsedTime xTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Motors
        LF = getMotor("FL"); LB = getMotor("BL");
        RF = getMotor("FR"); RB = getMotor("BR");
        intake = getMotor("intake"); outtakeL = getMotor("outtakeL"); outtakeR = getMotor("outtakeR");
        spinner = getMotor("spinner");
        lifter = getServo("lifter"); closer = getServo("closer");

        // Directions
        if (LF != null) LF.setDirection(DcMotor.Direction.REVERSE);
        if (LB != null) LB.setDirection(DcMotor.Direction.REVERSE);
        if (RF != null) RF.setDirection(DcMotor.Direction.FORWARD);
        if (RB != null) RB.setDirection(DcMotor.Direction.FORWARD);
        if (spinner != null) spinner.setDirection(DcMotor.Direction.REVERSE);
        if (outtakeL != null) outtakeL.setDirection(DcMotor.Direction.REVERSE);
        if (outtakeR != null) outtakeR.setDirection(DcMotor.Direction.FORWARD);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        // Limit switch
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Spinner encoder reset
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Compute degrees to ticks
        ticksPer55Degrees = (int)((55.0 / 360.0) * TICKS_PER_REV * GEAR_RATIO);
        ticksPer30Degrees = (int)((30.0 / 360.0) * TICKS_PER_REV * GEAR_RATIO);

        lastTime = System.nanoTime();

        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ----------------- FIELD-CENTRIC DRIVE -----------------
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

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
            if (max > 1.0) { fl /= max; bl /= max; fr /= max; br /= max; }

            double speed = 1 - (0.6 * gamepad1.right_trigger);
            setMotorPower(LF, fl * speed);
            setMotorPower(LB, bl * speed);
            setMotorPower(RF, fr * speed);
            setMotorPower(RB, br * speed);

            // ----------------- LIMIT SWITCH LOGIC -----------------
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
            }
            if (limitActive) {
                if (limitTimer.seconds() < 0.5) spinner.setPower(0);
                else if (!forwardClearStage) {
                    int eightDegreeTicks = (int)((8.0 / 360.0) * TICKS_PER_REV);
                    spinner.setTargetPosition(eightDegreeTicks);
                    spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spinner.setPower(0.2);
                    if (!spinner.isBusy()) {
                        spinner.setPower(0);
                        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        forwardClearStage = true;
                        limitActive = false;
                    }
                }
                continue; // skip PID while limit switch active
            }

            // ----------------- SPINNER PID -----------------
            if (gamepad2.y && !lastY) targetTicks += ticksPer55Degrees;
            lastY = gamepad2.y;

            // ----------------- X BUTTON NON-BLOCKING SEQUENCE -----------------
            if (gamepad2.x && !lastX && xSeqState == XSequence.IDLE) {
                xSeqState = XSequence.SPIN_30;
            }
            lastX = gamepad2.x;

            switch (xSeqState) {
                case SPIN_30:
                    targetTicks += ticksPer30Degrees;
                    xTimer.reset();
                    xSeqState = XSequence.LIFTER_UP;
                    break;

                case LIFTER_UP:
                    if (lifter != null) lifter.setPosition(0.81); // lift up
                    xTimer.reset();
                    xSeqState = XSequence.WAIT;
                    break;

                case WAIT:
                    if (xTimer.seconds() >= 0.5) {
                        xSeqState = XSequence.LIFTER_DOWN;
                    }
                    break;

                case LIFTER_DOWN:
                    if (lifter != null) lifter.setPosition(0.65); // bring down
                    xSeqState = XSequence.IDLE;
                    break;

                case IDLE:
                default:
                    break;
            }

            runPID(targetTicks);

            // ----------------- MECHANISMS TOGGLES -----------------
            if (gamepad2.a && !lastIntakeButton) intakeOn = !intakeOn;
            lastIntakeButton = gamepad2.a;
            setMotorPower(intake, intakeOn ? 0.5 : 0);

            if (gamepad2.b && !lastOuttakeButton) outtakeOn = !outtakeOn;
            lastOuttakeButton = gamepad2.b;
            setMotorPower(outtakeL, outtakeOn ? 0.55 : 0);
            setMotorPower(outtakeR, outtakeOn ? 0.55 : 0);

            // Only allow D-pad lifter control if X sequence is idle
            if (xSeqState == XSequence.IDLE) {
                if (gamepad2.dpad_up && !lastLifterButton) lifterOn = !lifterOn;
                lastLifterButton = gamepad2.dpad_up;
                setServoPosition(lifter, lifterOn ? 0.81 : 0.65);
            }

            if (gamepad2.dpad_down && !lastCloserButton) closerOn = !closerOn;
            lastCloserButton = gamepad2.dpad_down;
            setServoPosition(closer, closerOn ? 0.6 : 0.48);

            // ----------------- TELEMETRY -----------------
            telemetry.addData("Yaw", Math.toDegrees(yaw));
            telemetry.addData("Spinner Pos", spinner.getCurrentPosition());
            telemetry.addData("Spinner Target", targetTicks);
            telemetry.addData("Limit Active", limitActive);
            telemetry.addData("Intake", intakeOn);
            telemetry.addData("Outtake", outtakeOn);
            telemetry.addData("Lifter", lifterOn);
            telemetry.addData("X Sequence", xSeqState);
            telemetry.update();
        }
    }

    private void runPID(int target) {
        if (spinner == null) return;
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        dt = Math.max(0.001, Math.min(dt, 0.05));
        lastTime = now;

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

    private double getYaw() {
        if (imu == null) return 0;
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.dcMotor.get(name); } catch (Exception e) { return null; }
    }

    private Servo getServo(String name) {
        try { return hardwareMap.servo.get(name); } catch (Exception e) { return null; }
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (motor != null) motor.setPower(power);
    }

    private void setServoPosition(Servo servo, double pos) {
        if (servo != null) servo.setPosition(pos);
    }
}
