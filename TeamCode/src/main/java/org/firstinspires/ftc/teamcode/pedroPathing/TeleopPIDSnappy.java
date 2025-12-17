// -------------- FULL FILE WITH 40° X-SEQUENCE ----------------
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

@TeleOp(name = "ASNAPPYField-Centric Drive + Mechanisms + Limit Switch PID + X Spin 40° Lifter Sequence")
public class TeleopPIDSnappy extends LinearOpMode {

    private DcMotor LF, RF, LB, RB;
    private DcMotor intake, outtakeL, outtakeR, spinner;
    private Servo lifter, closer;
    private IMU imu;

    private DigitalChannel limitSwitch;
    private boolean limitActive = false;
    private boolean forwardClearStage = false;
    private ElapsedTime limitTimer = new ElapsedTime();

    private boolean lastIntakeButton = false, intakeOn = false;
    private boolean lastOuttakeButton = false, outtakeOn = false;
    private boolean lastLifterButton = false, lifterOn = false;
    private boolean lastCloserButton = false, closerOn = false;
    private boolean lastInvert = false, lastYawReset = false, lastY = false;
    private boolean lastX = false;
    private boolean invertedControls = false;
    private ElapsedTime buttonDelay = new ElapsedTime();

    double kP = 0.006, kI = 0, kD = 0.000003;
    double integral = 0, lastError = 0, integralMax = 3000;

    private boolean pidEnabled = true;
    private boolean lastPidToggle = false;

    private int targetTicks = 0;
    double TICKS_PER_REV = 753.2, GEAR_RATIO = 1.0;
    private int ticksPer40Degrees;
    private int ticksPer55Degrees;
    private long lastTime;

    // ---------------- X Button Sequence ----------------
    private enum XSequence { IDLE, SPIN_BACK, LIFTER_UP, WAIT, SPIN_FORWARD }
    private XSequence xSeqState = XSequence.IDLE;
    private ElapsedTime xTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        LF = getMotor("FL"); LB = getMotor("BL");
        RF = getMotor("FR"); RB = getMotor("BR");
        intake = getMotor("intake"); outtakeL = getMotor("outtakeL"); outtakeR = getMotor("outtakeR");
        spinner = getMotor("spinner");
        lifter = getServo("lifter"); closer = getServo("closer");

        if (LF != null) LF.setDirection(DcMotor.Direction.REVERSE);
        if (LB != null) LB.setDirection(DcMotor.Direction.REVERSE);
        if (RF != null) RF.setDirection(DcMotor.Direction.FORWARD);
        if (RB != null) RB.setDirection(DcMotor.Direction.FORWARD);
        if (spinner != null) spinner.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPer55Degrees = (int)((55.0 / 360.0) * TICKS_PER_REV);
        ticksPer40Degrees = (int)((40.0 / 360.0) * TICKS_PER_REV);

        lastTime = System.nanoTime();

        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ----------------- DRIVE -----------------
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
            double cosA = Math.cos(-yaw), sinA = Math.sin(-yaw);
            double tx = lx * cosA - ly * sinA;
            double ty = lx * sinA + ly * cosA;

            double fl = ty + tx + rx;
            double bl = ty - tx + rx;
            double fr = ty - tx - rx;
            double br = ty + tx - rx;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
            if (max > 1.0) { fl/=max; bl/=max; fr/=max; br/=max; }

            double speed = 1 - (0.6 * gamepad1.right_trigger);
            setMotorPower(LF, fl * speed);
            setMotorPower(LB, bl * speed);
            setMotorPower(RF, fr * speed);
            setMotorPower(RB, br * speed);

            // ----------------- LIMIT SWITCH -----------------
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
                if (limitTimer.seconds() < 0.3) spinner.setPower(0);
                else if (!forwardClearStage) {
                    int eightTicks = (int)((8.0/360.0)*TICKS_PER_REV);
                    spinner.setTargetPosition(eightTicks);
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
                continue;
            }

            // ----------------- Y BUTTON = +55° ----------------
            if (gamepad2.y && !lastY) targetTicks += ticksPer55Degrees;
            lastY = gamepad2.y;

            // ----------------- X BUTTON = 40° BACK → LIFT → 40° FORWARD ----------------
            if (gamepad2.x && !lastX && xSeqState == XSequence.IDLE) {
                xSeqState = XSequence.SPIN_BACK;
            }
            lastX = gamepad2.x;

            switch (xSeqState) {

                case SPIN_BACK:
                    targetTicks -= ticksPer40Degrees;   // BACKWARD 40°
                    xTimer.reset();
                    xSeqState = XSequence.LIFTER_UP;
                    break;

                case LIFTER_UP:
                    lifter.setPosition(0.81);
                    xTimer.reset();
                    xSeqState = XSequence.WAIT;
                    break;

                case WAIT:
                    if (xTimer.seconds() >= 0.5) {
                        xSeqState = XSequence.SPIN_FORWARD;
                    }
                    break;

                case SPIN_FORWARD:
                    targetTicks += ticksPer40Degrees;   // FORWARD 40°
                    lifter.setPosition(0.65);
                    xSeqState = XSequence.IDLE;
                    break;

                case IDLE:
                default:
                    break;
            }

            // ----------------- PID -----------------
            if (pidEnabled) runPID(targetTicks);
            else spinner.setPower(0);

            // ----------------- MECHANISMS -----------------
            if (gamepad2.a && !lastIntakeButton) intakeOn = !intakeOn;
            lastIntakeButton = gamepad2.a;
            setMotorPower(intake, intakeOn ? 0.5 : 0);

            if (gamepad2.b && !lastOuttakeButton) outtakeOn = !outtakeOn;
            lastOuttakeButton = gamepad2.b;
            setMotorPower(outtakeL, outtakeOn ? -0.55 : 0);
            setMotorPower(outtakeR, outtakeOn ? 0.55 : 0);

            if (gamepad2.dpad_down && !lastCloserButton) closerOn = !closerOn;
            lastCloserButton = gamepad2.dpad_down;
            setServoPosition(closer, closerOn ? 0.6 : 0.48);

            if (xSeqState == XSequence.IDLE) {
                if (gamepad2.dpad_up && !lastLifterButton) lifterOn = !lifterOn;
                lastLifterButton = gamepad2.dpad_up;
                setServoPosition(lifter, lifterOn ? 0.82 : 0.65);
            }

            telemetry.addData("Yaw", Math.toDegrees(yaw));
            telemetry.addData("Spinner Pos", spinner.getCurrentPosition());
            telemetry.addData("Spinner Target", targetTicks);
            telemetry.addData("X Seq", xSeqState);
            telemetry.update();
        }
    }

    private void runPID(int target) {
        long now = System.nanoTime();
        double dt = Math.max(0.001, Math.min((now - lastTime)/1e9, 0.05));
        lastTime = now;

        double pos = spinner.getCurrentPosition();
        double error = target - pos;
        integral += error * dt;
        integral = Math.max(-integralMax, Math.min(integral, integralMax));
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP*error + kI*integral + kD*derivative;
        output = Math.max(-0.3, Math.min(output, 0.3));
        spinner.setPower(output);
    }

    private double getYaw() {
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        return a.getYaw(AngleUnit.RADIANS);
    }

    private DcMotor getMotor(String n){ try{return hardwareMap.dcMotor.get(n);}catch(Exception e){return null;} }
    private Servo getServo(String n){ try{return hardwareMap.servo.get(n);}catch(Exception e){return null;} }
    private void setMotorPower(DcMotor m,double p){ if(m!=null)m.setPower(p); }
    private void setServoPosition(Servo s,double p){ if(s!=null)s.setPosition(p); }
}
