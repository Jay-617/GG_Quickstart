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

@TeleOp(name = "Field-Centric Drive + Mechanisms (Stable Spinner)")
public class TeleopFCStable extends LinearOpMode {

    // Drive motors
    private DcMotor LF, RF, LB, RB;

    // Mechanisms
    private DcMotor intake, outtakeL, outtakeR, spinner;
    private Servo lifter, closer;

    // Button states
    private boolean lastIntake = false, lastOuttake = false, lastLifter = false, lastCloser = false;
    private boolean lastInvert = false, lastSpinnerButton = false, lastYawReset = false;

    private int counterIntake = 1, counterOuttake = 1, counterLifter = 1, counterCloser = 1;
    private boolean invertedControls = false;

    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime buttonDelay = new ElapsedTime();

    // Constants
    static final double SPIN_SPEED = 0.6;
    static final int SPINNER_TICKS = 800;

    private double initialYaw = 0; // Original yaw at match start

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

        // --- Spinner encoder setup ---
        if (spinner != null) {
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("✅ Ready to start");
        telemetry.update();
        waitForStart();

        // Record initial yaw for field-centric orientation
        initialYaw = getYaw();

        while (opModeIsActive()) {

            // --- Joystick inputs ---
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y; // forward
            double rx = gamepad1.right_stick_x; // rotation

            // --- Invert controls toggle (Gamepad1 B) ---
            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
                invertedControls = !invertedControls;
                buttonDelay.reset();
            }
            lastInvert = gamepad1.b;
            if (invertedControls) { lx = -lx; ly = -ly; }

            // --- Reset yaw (Gamepad1 Y) ---
            if (gamepad1.y && !lastYawReset && buttonDelay.seconds() > 0.3) {
                initialYaw = getYaw();
                buttonDelay.reset();
            }
            lastYawReset = gamepad1.y;

            // --- FIELD-CENTRIC MOVEMENT ---
            double currentYaw = getYaw();
            double yawOffset = currentYaw - initialYaw; // How much robot has turned
            double cosA = Math.cos(-yawOffset); // Negative to rotate joystick relative to field
            double sinA = Math.sin(-yawOffset);

            double tempX = lx * cosA - ly * sinA;
            double tempY = lx * sinA + ly * cosA;

            // --- Mecanum drive powers ---
            double frontLeftPower  = tempY + tempX + rx;
            double backLeftPower   = tempY - tempX + rx;
            double frontRightPower = tempY - tempX - rx;
            double backRightPower  = tempY + tempX - rx;

            // Normalize powers
            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            // --- Scaled drive (Right Trigger) ---
            double speed = 1 - (0.6 * gamepad1.right_trigger);


            setMotorPower(LF, frontLeftPower * speed);
            setMotorPower(LB, backLeftPower * speed);
            setMotorPower(RF, frontRightPower * speed);
            setMotorPower(RB, backRightPower * speed);

//            // --- Spinner Control ---
//            boolean spinnerPressed = gamepad2.y;
//            boolean spinnerActive = spinner != null &&
//                    spinner.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
//                    spinner.isBusy();
//
//            if (spinnerPressed && !lastSpinnerButton && !spinnerActive && spinner != null) {
//                int target = spinner.getCurrentPosition() + (SPINNER_TICKS);
//                spinner.setTargetPosition(target);
//                spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                spinner.setPower(SPIN_SPEED);
//            }
//
//            if (spinner != null &&
//                    spinner.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
//                    !spinner.isBusy()) {
//                spinner.setPower(0);
//                spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//
//            lastSpinnerButton = spinnerPressed;

            // Spinner forward (Y)
            if (gamepad2.y) EncoderSpinner(SPIN_SPEED, 7.5, .6);

            // --- Intake Toggle ---
            if (gamepad2.a && !lastIntake) counterIntake++;
            lastIntake = gamepad2.a;
            setMotorPower(intake, (counterIntake % 2 == 0) ? 0.5 : 0);

            // --- Outtake Toggle ---
            if (gamepad2.b && !lastOuttake) counterOuttake++;
            lastOuttake = gamepad2.b;
            double outPower = (counterOuttake % 2 == 0) ? 0.53 : 0;
            double outPower2 = (counterOuttake % 2 == 0) ? 0.55 : 0;
            setMotorPower(outtakeL, outPower2);
            setMotorPower(outtakeR, outPower);

            // --- Lifter Toggle ---
            if (gamepad2.dpad_up && !lastLifter) counterLifter++;
            lastLifter = gamepad2.dpad_up;
            setServoPosition(lifter, (counterLifter % 2 == 0) ? 0.81 : 0.65);

            // --- Closer Toggle ---
            if (gamepad2.dpad_down && !lastCloser) counterCloser++;
            lastCloser = gamepad2.dpad_down;
            setServoPosition(closer, (counterCloser % 2 == 0) ? 0.6 : 0.48);

            // --- Telemetry ---
            telemetry.addData("Yaw (deg)", Math.toDegrees(currentYaw));
            telemetry.addData("Field-Centric Active", true);
            telemetry.addData("Inverted Controls", invertedControls);
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
        //250
        sleep(300);
    }
}
