package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "FieldCentricDriveTELOP (Safe Version)")
public class TeleopFieldCentric extends LinearOpMode {

    // Drive motors
    public DcMotor LF, RF, LB, RB;

    // Mechanisms
    public DcMotor intake, outtakeL, outtakeR, spinner;
    public Servo lifter, closer;

    // Toggles and counters
    int counter = 1;
    int counter_b = 1;
    int counter_c = 1;
    int counter_x = 1;
    int counterC = 1;
    int counterL = 1;

    // Button state tracking
    boolean lastB = false;
    boolean lastA = false;
    boolean lastY = false;
    boolean lastX = false;
    boolean lastC = false;
    boolean lastL = false;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 7.1;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double SPIN_SPEED = 0.6;

    @Override
    public void runOpMode() {
        // --- Safe hardware mapping ---
        LF = getMotor("FL");
        LB = getMotor("BL");
        RF = getMotor("FR");
        RB = getMotor("BR");

        intake = getMotor("intake");
        outtakeL = getMotor("outtakeL");
        outtakeR = getMotor("outtakeR");
        spinner = getMotor("spinner");

        lifter = getServo("lifter");
        closer = getServo("closer");

        // --- Display hardware status ---
        telemetry.addLine("⚙️ Hardware Check:");
        checkDevice(LF, "FL motor");
        checkDevice(RF, "FR motor");
        checkDevice(LB, "BL motor");
        checkDevice(RB, "BR motor");
        checkDevice(intake, "Intake motor");
        checkDevice(outtakeL, "OuttakeL motor");
        checkDevice(outtakeR, "OuttakeR motor");
        checkDevice(spinner, "Spinner motor");
        checkDevice(lifter, "Lifter servo");
        checkDevice(closer, "Closer servo");
        telemetry.update();
        sleep(2000);

        // --- Only set directions if motors exist ---
        if (LF != null) LF.setDirection(DcMotor.Direction.REVERSE);
        if (RF != null) RF.setDirection(DcMotor.Direction.FORWARD);
        if (LB != null) LB.setDirection(DcMotor.Direction.REVERSE);
        if (RB != null) RB.setDirection(DcMotor.Direction.FORWARD);
        if (spinner != null) spinner.setDirection(DcMotor.Direction.REVERSE);
        if (outtakeR != null) outtakeR.setDirection(DcMotor.Direction.FORWARD);
        if (outtakeL != null) outtakeL.setDirection(DcMotor.Direction.REVERSE);
        if (intake != null) intake.setDirection(DcMotor.Direction.FORWARD);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new
                RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Encoder setup for spinner
        if (spinner != null) {
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("✅ Ready to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

//        double left, right, drive, turn, max;

        while (opModeIsActive()) {

            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;


            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);


            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);


            if (gamepadRateLimit.hasExpired() && gamepad1.y) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }


            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);


            RB.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            LB.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            RF.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            LF.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
//            // Normal drive
//            drive = -gamepad1.left_stick_y;
//            turn = gamepad1.right_stick_x;
//
//            left = drive + turn;
//            right = drive - turn;
//
//            max = Math.max(Math.abs(left), Math.abs(right));
//            if (max > 1.0) {
//                left /= max;
//                right /= max;
//            }
//
//            setMotorPower(LF, left);
//            setMotorPower(LB, left);
//            setMotorPower(RF, right);
//            setMotorPower(RB, right);

            // Outtake toggle (B)
            boolean currentB = gamepad2.b;
            if (currentB && !lastB) counter++;
            lastB = currentB;
            double outPower = (counter % 2 == 0) ? 0.6 : 0;
            setMotorPower(outtakeR, outPower);
            setMotorPower(outtakeL, outPower);

            // Intake toggle (A)
            boolean currentA = gamepad2.a;
            if (currentA && !lastA) counter_b++;
            lastA = currentA;
            setMotorPower(intake, (counter_b % 2 == 0) ? .5 : 0);

            // Spinner forward (Y)
            if (gamepad2.y) EncoderSpinner(SPIN_SPEED, 8, .6);

//            // Spinner reverse (X)
//            boolean currentX = gamepad2.x;
//            if (currentX && !lastX) counter_x++;
//            lastX = currentX;
//            if (counter_x % 2 == 0) {
//                setMotorPower(outtakeR, 0.3);
//                setMotorPower(outtakeL, -0.3);
//            }

            // Lifter toggle (D-pad up)
            boolean currentL = gamepad2.dpad_up;
            if (currentL && !lastL) counterL++;
            lastL = currentL;
            setServoPosition(lifter, (counterL % 2 == 0) ? 0.8 : 0.65);

            // Closer toggle (D-pad down)
            boolean currentC = gamepad2.dpad_down;
            if (currentC && !lastC) counterC++;
            lastC = currentC;
            setServoPosition(closer, (counterC % 2 == 0) ? 0.6 : 0.48);

//            // Left strafe
//            if (gamepad1.left_bumper) {
//                double strafePower = 0.6;
//                setMotorPower(LF, -strafePower);
//                setMotorPower(RF, strafePower);
//                setMotorPower(LB, strafePower);
//                setMotorPower(RB, -strafePower);
//            }
//            //right strafe
//            if (gamepad1.right_bumper) {
//                double strafePower = 0.6;
//                setMotorPower(LF, strafePower);
//                setMotorPower(RF, -strafePower);
//                setMotorPower(LB, -strafePower);
//                setMotorPower(RB, strafePower);
//            }
        }
    }

    // --- Helper functions ---

    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (Exception e) {
            telemetry.addData("⚠️ Missing Motor", name);
            return null;
        }
    }

    private Servo getServo(String name) {
        try {
            return hardwareMap.servo.get(name);
        } catch (Exception e) {
            telemetry.addData("⚠️ Missing Servo", name);
            return null;
        }
    }

    private void setMotorPower(DcMotor motor, double power) {
        if (motor != null) motor.setPower(power);
    }

    private void setServoPosition(Servo servo, double pos) {
        if (servo != null) servo.setPosition(pos);
    }

    private void checkDevice(Object device, String name) {
        telemetry.addData(name, (device != null) ? "✅ Found" : "❌ MISSING");
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
