package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleopFieldCentric.COUNTS_PER_INCH;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Field-Centric Drive + Mechanisms (Stable Spinner + PID Fixed)")
public class TeleopFCStablePIDFixed2 extends LinearOpMode {

    // Drive motors
    private DcMotor LF, RF, LB, RB;

    // Mechanisms
    private DcMotor intake, outtakeL, outtakeR, spinner;
    private Servo lifter, closer;

    // Button states
    private boolean lastIntake = false, lastOuttake = false, lastLifter = false, lastCloser = false;
    private boolean lastInvert = false, lastYawReset = false;
    private boolean lastY = false; // Y button edge

    private int counterIntake = 1, counterOuttake = 1, counterLifter = 1, counterCloser = 1;
    private boolean invertedControls = false;

    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime buttonDelay = new ElapsedTime();

    private double initialYaw = 0;

    // PID constants
    double kP = 0.006;
    double kI = 0;
    double kD = 0.000003;
    double integral = 0;
    double lastError = 0;
    double integralMax = 3000;

    double TICKS_PER_REV = 753.2;
    double GEAR_RATIO = 1.0;

    // Spinner target in ticks
    int targetTicks = 0;
    int ticksPer58Degrees;
    private long lastTime;

    @Override
    public void runOpMode() {

        // Hardware map
        LF = getMotor("FL"); LB = getMotor("BL");
        RF = getMotor("FR"); RB = getMotor("BR");

        intake = getMotor("intake");
        outtakeL = getMotor("outtakeL");
        outtakeR = getMotor("outtakeR");
        spinner = getMotor("spinner");

        lifter = getServo("lifter");
        closer = getServo("closer");

        // Directions
        if (LF != null) LF.setDirection(DcMotor.Direction.REVERSE);
        if (LB != null) LB.setDirection(DcMotor.Direction.REVERSE);
        if (RF != null) RF.setDirection(DcMotor.Direction.FORWARD);
        if (RB != null) RB.setDirection(DcMotor.Direction.FORWARD);
        if (spinner != null) spinner.setDirection(DcMotor.Direction.REVERSE);
        if (outtakeL != null) outtakeL.setDirection(DcMotor.Direction.REVERSE);
        if (outtakeR != null) outtakeR.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Reset spinner encoder
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Precompute encoder ticks per 58 degrees
        ticksPer58Degrees = (int)((58 / 360.0) * TICKS_PER_REV * GEAR_RATIO);

        lastTime = System.nanoTime();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        initialYaw = getYaw();

        while (opModeIsActive()) {

            // Joysticks
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // Invert controls
            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
                invertedControls = !invertedControls;
                buttonDelay.reset();
            }
            lastInvert = gamepad1.b;

            if (invertedControls) { lx = -lx; ly = -ly; }

            // Reset yaw
            if (gamepad1.y && !lastYawReset) imu.resetYaw();
            lastYawReset = gamepad1.y;

            // Field centric
            double yaw = getYaw();
            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);

            double tempX = lx * cosA - ly * sinA;
            double tempY = lx * sinA + ly * cosA;

            double fl = tempY + tempX + rx;
            double bl = tempY - tempX + rx;
            double fr = tempY - tempX - rx;
            double br = tempY + tempX - rx;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br)));
            if (max > 1.0) {
                fl /= max; bl /= max; fr /= max; br /= max;
            }

            double speed = 1 - (0.6 * gamepad1.right_trigger);

            setMotorPower(LF, fl * speed);
            setMotorPower(LB, bl * speed);
            setMotorPower(RF, fr * speed);
            setMotorPower(RB, br * speed);

            // Y button pressed — add 58° in ticks
            if (gamepad2.y && !lastY) {
                targetTicks += ticksPer58Degrees;
            }
            lastY = gamepad2.y;

            // PID forever
            runPIDHold(targetTicks);

            // Intake toggle
            if (gamepad2.a && !lastIntake) counterIntake++;
            lastIntake = gamepad2.a;
            setMotorPower(intake, (counterIntake % 2 == 0) ? 0.5 : 0);

            // Outtake toggle
            if (gamepad2.b && !lastOuttake) counterOuttake++;
            lastOuttake = gamepad2.b;
            setMotorPower(outtakeL, (counterOuttake % 2 == 0) ? 0.55 : 0);
            setMotorPower(outtakeR, (counterOuttake % 2 == 0) ? 0.53 : 0);

            // Lifter toggle
            if (gamepad2.dpad_up && !lastLifter) counterLifter++;
            lastLifter = gamepad2.dpad_up;
            setServoPosition(lifter, (counterLifter % 2 == 0) ? 0.81 : 0.65);

            // Closer toggle
            if (gamepad2.dpad_down && !lastCloser) counterCloser++;
            lastCloser = gamepad2.dpad_down;
            setServoPosition(closer, (counterCloser % 2 == 0) ? 0.6 : 0.48);

            telemetry.addData("Yaw", Math.toDegrees(yaw));
            telemetry.addData("Spinner Target Ticks", targetTicks);
            telemetry.addData("Spinner Pos", spinner.getCurrentPosition());
            telemetry.update();
        }
    }

    // PID function — spinner target in ticks
    public void runPIDHold(int targetTicks) {
        if (spinner == null) return;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        if (dt < 0.001) dt = 0.001;
        if (dt > 0.1) dt = 0.1;
        lastTime = now;

        double current = spinner.getCurrentPosition();
        double error = targetTicks - current;

        integral += error * dt;
        integral = Math.max(-integralMax, Math.min(integral, integralMax));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;

        // Max speed limit
        double maxSpeed = 0.3;
        output = Math.max(-maxSpeed, Math.min(output, maxSpeed));

        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setPower(output);
    }

    // Helpers
    private double getYaw() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.dcMotor.get(name); }
        catch (Exception e) { return null; }
    }

    private Servo getServo(String name) {
        try { return hardwareMap.servo.get(name); }
        catch (Exception e) { return null; }
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (motor != null) motor.setPower(power);
    }

    private void setServoPosition(Servo servo, double pos) {
        if (servo != null) servo.setPosition(pos);
    }
}
