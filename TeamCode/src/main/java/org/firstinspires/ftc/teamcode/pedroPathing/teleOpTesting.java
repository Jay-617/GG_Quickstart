package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
@TeleOp
public class teleOpTesting extends LinearOpMode {

    // Drive motors
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    // Mechanisms
    public DcMotor intake = null;
    public DcMotor outtakeL = null;
    public DcMotor outtakeR = null;
    public DcMotor spinner = null;
    public Servo lifter = null;
    public Servo closer = null;

    // Toggles and counters
    int counter = 1;    // outtake toggle
    int counter_b = 1;  // intake toggle
    int counter_c = 1;  // spinner forward toggle
    int counter_x = 1;  // spinner reverse toggle
    int counterC = 1;   // closer servo
    int counterL = 1;   // lifter servo

    // Button state tracking
    boolean lastB = false;
    boolean lastA = false;
    boolean lastY = false;
    boolean lastX = false;
    boolean lastC = false;
    boolean lastL = false;

    private ElapsedTime runtime = new ElapsedTime();

    // Encoder constants (goBILDA 5204-435 RPM motor)
    static final double COUNTS_PER_MOTOR_REV = 384.5; // Encoder counts per output revolution
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 7.1;  // Wheel diameter (inches)
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double SPIN_SPEED = 0.6;

    @Override
    public void runOpMode() {
        // Initialize hardware
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        lifter = hardwareMap.servo.get("lifter");
        closer = hardwareMap.servo.get("closer");

        intake = hardwareMap.dcMotor.get("intake");
        outtakeL = hardwareMap.dcMotor.get("outtakeL");
        outtakeR = hardwareMap.dcMotor.get("outtakeR");
        spinner = hardwareMap.dcMotor.get("spinner");

        // Motor directions
//        LF.setDirection(DcMotor.Direction.FORWARD);
//        RF.setDirection(DcMotor.Direction.FORWARD);
//        LB.setDirection(DcMotor.Direction.FORWARD);
//        RB.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.REVERSE);

        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        outtakeL.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // IMU setup
        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

//        // Initialize servos
//        closer.setPosition(0.5);
//        lifter.setPosition(0.7);
//        sleep(100);

        // Encoder setup for spinner
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting Encoder", spinner.getCurrentPosition());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

            // Reset yaw with Y
            if (gamepadRateLimit.hasExpired() && gamepad1.y) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            RB.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);
            LB.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            RF.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            LF.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);

            // --- Mechanism controls ---

            // Toggle outtake (B)
            boolean currentB = gamepad2.b;
            if (currentB && !lastB) counter++;
            lastB = currentB;
            if (counter % 2 == 0) {
                outtakeR.setPower(.6);
                outtakeL.setPower(.6);
            } else {
                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }

            // Toggle intake (A)
            boolean currentA = gamepad2.a;
            if (currentA && !lastA) counter_b++;
            lastA = currentA;
            intake.setPower(counter_b % 2 == 0 ? 0.7 : 0);

            // Spinner forward (Y)
            if (gamepad2.y){
                EncoderSpinner(SPIN_SPEED, 5,.6);
            }

            boolean currentX = gamepad2.x;
//            if (currentX && !lastX) counter_x++;
//            lastX = currentX;

            if (counter_x % 2 != 0) {
//
                outtakeR.setPower(.3);
                outtakeL.setPower(-.3);

            } else {

                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }


//            boolean currentY = gamepad2.y;
//            if (currentY && !lastY) counter_c++;
//            lastY = currentY;
//
//            // Spinner reverse (X)
//            boolean currentX = gamepad2.x;
//            if (currentX && !lastX) counter_x++;
//            lastX = currentX;
//
//            if (counter_c % 2 == 0 && counter_x % 2 != 0) {
//                spinner.setDirection(DcMotorSimple.Direction.FORWARD);
//                spinner.setPower(0.1);
//                outtakeR.setPower(.1);
//                outtakeL.setPower(.1);
//            } else if (counter_x % 2 == 0 && counter_c % 2 != 0) {
//                spinner.setDirection(DcMotorSimple.Direction.REVERSE);
//                spinner.setPower(0.1);
//                outtakeR.setPower(.1);
//                outtakeL.setPower(.1);
//            } else {
//                spinner.setPower(0);
//                outtakeR.setPower(0);
//                outtakeL.setPower(0);
//            }

            // Lifter toggle (D-pad Up)
            boolean currentL = gamepad2.dpad_up;
            if (currentL && !lastL) counterL++;
            lastL = currentL;
            lifter.setPosition(counterL % 2 == 0 ? 0.8 : 0.7);

            // Closer toggle (D-pad Down)
            boolean currentC = gamepad2.dpad_down;
            if (currentC && !lastC) counterC++;
            lastC = currentC;
            closer.setPosition(counterC % 2 == 0 ? 0.6 : 0.5);
        }

        // Example encoder use after teleop loop (optional)
        EncoderSpinner(1.0, 3.0, 1.0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(200);
    }

    /**
     * Moves the spinner motor using encoder control for a given distance.
     */
    public void EncoderSpinner(double speed, double distanceInches, double timeoutS) {
        int target;

        if (opModeIsActive()) {
            target = spinner.getCurrentPosition() + (int)(distanceInches * COUNTS_PER_INCH);
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
            sleep(250);
        }
    }
}
