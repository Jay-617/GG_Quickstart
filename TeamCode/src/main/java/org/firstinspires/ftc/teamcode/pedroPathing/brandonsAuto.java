package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class brandonsAuto extends LinearOpMode {

    // Motors
    private DcMotor LF, RF, LB, RB, intake, outtakeL, outtakeR, spinner;

    private final ElapsedTime runtime = new ElapsedTime();

    // Constants
    static final double COUNTS_PER_MOTOR_REV = 312;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 1.85;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {

        // ---------------- MAP MOTORS ----------------
        RF = hardwareMap.get(DcMotor.class, "FR");
        LF = hardwareMap.get(DcMotor.class, "FL");
        RB = hardwareMap.get(DcMotor.class, "BR");
        LB = hardwareMap.get(DcMotor.class, "BL");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeL = hardwareMap.get(DcMotor.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotor.class, "outtakeR");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        // ---------------- MOTOR SETTINGS ----------------
        // Directions
        RF.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        spinner.setDirection(DcMotor.Direction.FORWARD);
        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        outtakeL.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Encoders
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake behavior for all motors
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        // ---------------- BEGIN AUTO ----------------
        StrafeRight(0.2, 1000);  // example


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // ---------------- MOVEMENT METHODS ----------------
    public void Drive(double speed, long time) {
        RF.setPower(speed);
        LF.setPower(speed);
        RB.setPower(speed);
        LB.setPower(speed);
        sleep(time);
        stopDrive();
    }

    public void StrafeLeft(double speed, long time) {
        RF.setPower(speed);
        LF.setPower(-speed);
        RB.setPower(-speed);
        LB.setPower(speed);
        sleep(time);
        stopDrive();
    }

    public void StrafeRight(double speed, long time) {
        RF.setPower(speed);
        LF.setPower(-speed);
        RB.setPower(speed);
        LB.setPower(-speed);
        sleep(time);
        stopDrive();
    }

    public void RotateLeft(double speed, long time) {
        RF.setPower(speed);
        LF.setPower(-speed);
        RB.setPower(speed);
        LB.setPower(-speed);
        sleep(time);
        stopDrive();
    }

    public void RotateRight(double speed, long time) {
        RF.setPower(-speed);
        LF.setPower(speed);
        RB.setPower(-speed);
        LB.setPower(speed);
        sleep(time);
        stopDrive();
    }

    // ---------------- MECHANISM METHODS ----------------
    public void Intake(double speed, long time) {
        intake.setPower(speed);
        sleep(time);
        intake.setPower(0);  // BRAKE hold
    }

    public void Outtake(double speed, long time) {
        outtakeL.setPower(speed);
        outtakeR.setPower(speed);
        sleep(time);
        outtakeL.setPower(0);
        outtakeR.setPower(0);
    }

    public void Spinner(double speed, long time) {
        spinner.setPower(speed);
        sleep(time);
        spinner.setPower(0);
    }

    // ---------------- HELPER ----------------
    private void stopDrive() {
        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
    }
}
