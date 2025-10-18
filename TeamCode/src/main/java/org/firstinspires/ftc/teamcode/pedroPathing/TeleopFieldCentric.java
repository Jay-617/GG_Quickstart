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
///
@TeleOp public class TeleopFieldCentric extends LinearOpMode {

    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    public DcMotor Spinner = null;
    IMU imu;
    /// //////////////////////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        Spinner = hardwareMap.get(DcMotor.class, "Spinner");

        // Set motor directions
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        Spinner.setDirection(DcMotor.Direction.FORWARD);

        // Use encoders for better control
//        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU with correct orientation
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("Ready. Press Play to start");
        telemetry.update();
/// ///
        waitForStart();
        if (isStopRequested()) return;








/// ////
        while (opModeIsActive()) {
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");

            if (gamepad1.a) {
                imu.resetYaw();
            }

            if (gamepad1.left_bumper) {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            telemetry.update();
            idle(); // Let the system handle other tasks
        }
    }

    /// //////////////////




    // Drive robot relative to the field using IMU heading
    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    // Set motor powers based on joystick inputs
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = Math.max(1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)))));

        double maxSpeed = 1.0; // Adjust for speed limit

        LF.setPower(maxSpeed * frontLeftPower / maxPower);
        RF.setPower(maxSpeed * frontRightPower / maxPower);
        LB.setPower(maxSpeed * backLeftPower / maxPower);
        RB.setPower(maxSpeed * backRightPower / maxPower);
    }
}
