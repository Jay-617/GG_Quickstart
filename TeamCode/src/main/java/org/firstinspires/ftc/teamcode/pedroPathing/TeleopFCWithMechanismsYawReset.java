package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentricDrive + Mechanisms fixed (Yaw Reset)")
public class TeleopFCWithMechanismsYawReset extends LinearOpMode {

    // Drive motors
    private DcMotor LF, RF, LB, RB;

    // Mechanisms
    private DcMotor intake, outtakeL, outtakeR, spinner;
    private Servo lifter, closer;

    // Button counters and states
    private int counterIntake = 1, counterOuttake = 1, counterLifter = 1, counterCloser = 1;
    private boolean lastIntake = false, lastOuttake = false, lastLifter = false, lastCloser = false;
    private boolean lastYawReset = false;

    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    // IMU field-centric zero reference
    private double zeroYaw = 0.0;

    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 7.1;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double SPIN_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        LF = getMotor("FL"); LB = getMotor("BL");
        RF = getMotor("FR"); RB = getMotor("BR");

        intake = getMotor("intake"); outtakeL = getMotor("outtakeL");
        outtakeR = getMotor("outtakeR"); spinner = getMotor("spinner");

        lifter = getServo("lifter"); closer = getServo("closer");

        // --- Motor directions ---
        if (LF != null) LF.setDirection(DcMotor.Direction.REVERSE);
        if (LB != null) LB.setDirection(DcMotor.Direction.REVERSE);
        if (RF != null) RF.setDirection(DcMotor.Direction.FORWARD);
        if (RB != null) RB.setDirection(DcMotor.Direction.FORWARD);
        if (spinner != null) spinner.setDirection(DcMotor.Direction.REVERSE);
        if (outtakeL != null) outtakeL.setDirection(DcMotor.Direction.REVERSE);
        if (outtakeR != null) outtakeR.setDirection(DcMotor.Direction.FORWARD);
        if (intake != null) intake.setDirection(DcMotor.Direction.FORWARD);

        // --- IMU setup ---
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Spinner encoder setup
        if (spinner != null) {
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("✅ Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // --- Reset yaw with gamepad1 A ---
            if (gamepad1.a && !lastYawReset) {
                zeroYaw = getYaw();
            }
            lastYawReset = gamepad1.a;

            // --- Joystick inputs ---
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y; // forward is negative y
            double rx = gamepad1.right_stick_x; // rotation

            // --- Field-centric calculations ---
            double yaw = getYaw() - zeroYaw; // apply zero reference
            // Wrap yaw to [-pi, pi] for smooth rotation
            yaw = ((yaw + Math.PI) % (2 * Math.PI)) - Math.PI;

            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);
            double tempX = lx * cosA - ly * sinA;
            double tempY = lx * sinA + ly * cosA;

            // --- Mecanum drive calculation ---
            double frontLeftPower  = tempY + tempX + rx;
            double backLeftPower   = tempY - tempX + rx;
            double frontRightPower = tempY - tempX - rx;
            double backRightPower  = tempY + tempX - rx;

            // Normalize powers
            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
            if (max > 1.0) {
                frontLeftPower  /= max;
                backLeftPower   /= max;
                frontRightPower /= max;
                backRightPower  /= max;
            }

            // Speed scaling
            double speed = 0.8 - (0.6 * gamepad1.right_trigger);
            setMotorPower(LF, frontLeftPower * speed);
            setMotorPower(LB, backLeftPower * speed);
            setMotorPower(RF, frontRightPower * speed);
            setMotorPower(RB, backRightPower * speed);

            // --- Mechanism toggles ---
            if (gamepad2.y) EncoderSpinner(SPIN_SPEED, 8, .6);

            if (gamepad2.a && !lastIntake) counterIntake++;
            lastIntake = gamepad2.a;
            setMotorPower(intake, (counterIntake % 2 == 0) ? 0.5 : 0);

            if (gamepad2.b && !lastOuttake) counterOuttake++;
            lastOuttake = gamepad2.b;
            double outPower = (counterOuttake % 2 == 0) ? 0.6 : 0;
            setMotorPower(outtakeL, outPower);
            setMotorPower(outtakeR, outPower);

            if (gamepad2.dpad_up && !lastLifter) counterLifter++;
            lastLifter = gamepad2.dpad_up;
            setServoPosition(lifter, (counterLifter % 2 == 0) ? 0.8 : 0.65);

            if (gamepad2.dpad_down && !lastCloser) counterCloser++;
            lastCloser = gamepad2.dpad_down;
            setServoPosition(closer, (counterCloser % 2 == 0) ? 0.6 : 0.48);

            // --- Telemetry ---
            telemetry.addData("Yaw (deg)", Math.toDegrees(yaw));
            telemetry.update();
        }
    }

    // --- Helper functions ---
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

    public void EncoderSpinner(double speed, double distanceInches, double timeoutS) {
        if (spinner == null) return;
        int target = spinner.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
        spinner.setTargetPosition(target);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        spinner.setPower(Math.abs(speed));

        while (opModeIsActive() && runtime.seconds() < timeoutS && spinner.isBusy()) {
            telemetry.addData("Running to", target);
            telemetry.addData("Currently at", spinner.getCurrentPosition());
            telemetry.update();
        }

        spinner.setPower(0);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(300);
    }
}
