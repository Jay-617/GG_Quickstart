package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name="aidenjavaclass", group="Robot")
public class javaclass extends LinearOpMode {

    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Initialize motors
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        // Set motor directions
        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- NORMAL DRIVE ---
            drive = -gamepad1.left_stick_y;   // Forward/backward
            turn  =  gamepad1.right_stick_x;  // Turning

            left  = drive + turn;
            right = drive - turn;

            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // --- STRAFE LEFT (Left Bumper) ---
            if (gamepad1.left_bumper) {
                double strafePower = 0.6; // Adjust for speed
                // Mecanum wheel strafe pattern (slide left)
                FR.setPower(strafePower);
                BR.setPower(-strafePower);
                FL.setPower(-strafePower);
                BL.setPower(strafePower);
            } else {
                // Standard tank drive
                FR.setPower(right);
                BR.setPower(right);
                FL.setPower(left);
                BL.setPower(left);
            }


            // --- STRAFE Right(Right Bumper) ---
            if (gamepad1.right_bumper) {
                double strafePower = 0.6; // Adjust for speed
                // Mecanum wheel strafe pattern (slide left)
                FR.setPower(-strafePower);
                BR.setPower(strafePower);
                FL.setPower(strafePower);
                BL.setPower(-strafePower);
            } else {
                // Standard tank drive
                FR.setPower(right);
                BR.setPower(right);
                FL.setPower(left);
                BL.setPower(left);
            }


            telemetry.addData("Drive", "L: %.2f R: %.2f", left, right);
            telemetry.update();
        }
    }
}
