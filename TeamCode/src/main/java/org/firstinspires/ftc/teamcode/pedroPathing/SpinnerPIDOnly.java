package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name = "Spinner PID ONLY")
public class SpinnerPIDOnly extends LinearOpMode {

    DcMotor spinner;

    // ===== ENCODER CONSTANTS =====
    static final int COUNTS_PER_REV = 8192;

    // ===== PID CONSTANTS =====
    static final double kP = 0.0026;
    static final double kI = 0.000015;
    static final double kD = 0.00009;

    static final double HOLD_POWER = 0.08;
    static final int DEADZONE = 4;
    static final int SLOW_ZONE = 220;

    // ===== PID STATE =====
    int spinnerTarget = 0;
    double integral = 0;
    double lastError = 0;

    @Override
    public void runOpMode() {

        spinner = hardwareMap.dcMotor.get("spinner");

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            int currentPos = spinner.getCurrentPosition();

            // ===== EXAMPLE TARGET =====
            // Change this number to whatever tick you want
            spinnerTarget = 2730;

            // ===== PID =====
            int error = spinnerTarget - currentPos;

            if (Math.abs(error) < 200) integral += error;
            else integral = 0;

            double derivative = error - lastError;

            double output =
                    (kP * error) +
                            (kI * integral) +
                            (kD * derivative);

            if (Math.abs(error) < SLOW_ZONE) output *= 0.5;
            if (Math.abs(error) <= DEADZONE)
                output = Math.signum(error) * HOLD_POWER;

            output = Math.max(-0.25, Math.min(0.25, output));

            spinner.setPower(output);
            lastError = error;

            // ===== TELEMETRY =====
            telemetry.addData("Target", spinnerTarget);
            telemetry.addData("Position", currentPos);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();
        }
    }
}