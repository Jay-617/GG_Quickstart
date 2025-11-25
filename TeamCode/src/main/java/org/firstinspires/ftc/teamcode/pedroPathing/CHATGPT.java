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
// --- Imports and package omitted for brevity ---
@TeleOp(name = "PRANAV'S AURA IS INSANE")
public class CHATGPT extends LinearOpMode {

    private DcMotor LF, RF, LB, RB;
    private DcMotor intake, outtakeL, outtakeR, spinner;
    private Servo lifter, closer;

    private boolean lastIntake = false, lastOuttake = false, lastLifter = false, lastCloser = false;
    private boolean lastInvert = false, lastSpinnerButton = false, lastYawReset = false;

    private int counterIntake = 0, counterOuttake = 0, counterLifter = 0, counterCloser = 0, counterSpinner = 0;
    private boolean invertedControls = false;

    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime buttonDelay = new ElapsedTime();

    static final double SPIN_SPEED = 0.6; // same as TeleopFCStable
    static final int SPINNER_TICKS = 800;

    private double initialYaw = 0;

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

        initialYaw = getYaw();

        while (opModeIsActive()) {

            // --- Joystick inputs ---
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // --- Invert toggle ---
            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
                invertedControls = !invertedControls;
                buttonDelay.reset();
            }
            lastInvert = gamepad1.b;
            if (invertedControls) { lx = -lx; ly = -ly; }

            // --- Reset IMU heading ---
            if (gamepad1.y && !lastYawReset && buttonDelay.seconds() > 0.3) {
                initialYaw = getYaw();
                buttonDelay.reset();
            }
            lastYawReset = gamepad1.y;

            // --- Field-centric drive ---
            double currentYaw = getYaw();
            double yawOffset = currentYaw - initialYaw;
            double cosA = Math.cos(-yawOffset);
            double sinA = Math.sin(-yawOffset);

            double tempX = lx * cosA - ly * sinA;
            double tempY = lx * sinA + ly * cosA;

            double fl = tempY + tempX + rx;
            double bl = tempY - tempX + rx;
            double fr = tempY - tempX - rx;
            double br = tempY + tempX - rx;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
            if (max > 1.0) { fl /= max; bl /= max; fr /= max; br /= max; }

            double speed = 1.0; // full power
            setMotorPower(LF, fl * speed);
            setMotorPower(LB, bl * speed);
            setMotorPower(RF, fr * speed);
            setMotorPower(RB, br * speed);

            // --- Spinner toggle using EncoderSpinner ---
            if (gamepad2.y && !lastSpinnerButton) {
                EncoderSpinner(SPIN_SPEED, 5.5, 0.6); // spins a set distance and stops automatically
            }
            lastSpinnerButton = gamepad2.y;


            // --- Intake toggle ---
            if (gamepad2.a && !lastIntake) counterIntake++;
            lastIntake = gamepad2.a;
            setMotorPower(intake, (counterIntake % 2 == 1) ? 0.5 : 0);

            // --- Outtake toggle ---o
            if (gamepad2.b && !lastOuttake) counterOuttake++;
            lastOuttake = gamepad2.b;
            double outPowerL = (counterOuttake % 2 == 1) ? 0.55 : 0; // left
            double outPowerR = (counterOuttake % 2 == 1) ? 0.65 : 0; // right
            setMotorPower(outtakeL, outPowerL);
            setMotorPower(outtakeR, outPowerR);


            // --- Lifter toggle ---
            if (gamepad2.dpad_up && !lastLifter) counterLifter++;
            lastLifter = gamepad2.dpad_up;
            if (counterLifter > 0) { // prevents initial movement
                setServoPosition(lifter, (counterLifter % 2 == 1) ? 0.81 : 0.65);
            }

            // --- Closer toggle ---
            if (gamepad2.dpad_down && !lastCloser) counterCloser++;
            lastCloser = gamepad2.dpad_down;
            if (counterCloser > 0) {
                setServoPosition(closer, (counterCloser % 2 == 1) ? 0.6 : 0.48);
            }

            // --- Telemetry ---
            telemetry.addData("Yaw (deg)", Math.toDegrees(currentYaw));
            telemetry.addData("Field-Centric Active", true);
            telemetry.update();
        }
    }

    // --- Helper functions ---
    private double getYaw() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }
    private DcMotor getMotor(String name) { try { return hardwareMap.dcMotor.get(name); } catch (Exception e) { return null; } }
    private Servo getServo(String name) { try { return hardwareMap.servo.get(name); } catch (Exception e) { return null; } }
    private void setMotorPower(DcMotor motor, double power) { if (motor != null) motor.setPower(power); }
    private void setServoPosition(Servo servo, double pos) { if (servo != null) servo.setPosition(pos); }

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
