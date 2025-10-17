package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teleOpTesting extends LinearOpMode {

    // Motors
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    public DcMotor intake = null;
    public DcMotor outtakeL = null;
    public DcMotor outtakeR = null;
    public DcMotor spinner = null;

    IMU imu;

    int counter = 1;
    int counter_b = 1;
    int counter_c = 1;

    // Track previous button state for toggles
    boolean lastB = false;
    boolean lastA = false;
    boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        intake = hardwareMap.dcMotor.get("intake");
        outtakeL = hardwareMap.dcMotor.get("outtakeL");
        outtakeR = hardwareMap.dcMotor.get("outtakeR");
        spinner = hardwareMap.dcMotor.get("spinner");

        // Set directions
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);

        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        outtakeL.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("Ready. Press Play to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine("Press Y to reset Yaw");
            telemetry.addLine("Hold X to drive in robot-relative mode");
            telemetry.addLine("Left joystick = movement, Right joystick = turning");

            // Reset yaw
            if (gamepad1.y) {
                imu.resetYaw();
            }

            // Driving logic
            if (gamepad1.x) {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            // Toggle outtake
            boolean currentB = gamepad2.b;
            if (currentB && !lastB) {
                counter++;
            }
            lastB = currentB;

            if (counter % 2 == 0) {
                outtakeR.setPower(1);
                outtakeL.setPower(1);
            } else {
                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }

            // Toggle intake
            boolean currentA = gamepad2.a;
            if (currentA && !lastA) {
                counter_b++;
            }
            lastA = currentA;

            if (counter_b % 2 == 0) {
                intake.setPower(.7);
            } else {
                intake.setPower(0);
            }

            // Toggle spinner
            boolean currentY = gamepad2.y;
            if (currentY && !lastY) {
                counter_c++;
            }
            lastY = currentY;

            if (counter_c % 2 == 0) {
                spinner.setPower(0.1);
                sleep(1000); // Pauses for 1 second
            } else {
                spinner.setPower(0); // ✅ Correct motor to stop
            }




            telemetry.update();
            idle(); // Let the system handle background tasks
        }
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)))));

        double maxSpeed = 1.0;

        LF.setPower(maxSpeed * frontLeftPower / maxPower);
        RF.setPower(maxSpeed * frontRightPower / maxPower);
        LB.setPower(maxSpeed * backLeftPower / maxPower);
        RB.setPower(maxSpeed * backRightPower / maxPower);
    }
}