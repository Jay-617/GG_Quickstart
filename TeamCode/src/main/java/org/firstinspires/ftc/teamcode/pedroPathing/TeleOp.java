package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;   // <-- required
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group = "A1")
public class TeleOP extends LinearOpMode {

    // ======== TUNE THESE ========
    private static final double DRIVE_POWER_SCALE = 1.0;   // overall drive scale
    private static final double INTAKE_POWER      = 1.0;   // intake motor power when ON
    private static final double OUTTAKE_POWER     = 1.0;   // outtake motor power when ON

    // Sorting mechanism positions (adjust for your linkage/servo)
    private static final double SORT_LEFT    = 0.15;
    private static final double SORT_CENTER = 0.50;
    private static final double SORT_RIGHT  = 0.85;

    @Override
    public void runOpMode() throws InterruptedException {

        // ======== Drivetrain (4 DC motors) ========
        DcMotor fL = hardwareMap.dcMotor.get("fL");
        DcMotor bL = hardwareMap.dcMotor.get("bL");
        DcMotor fR = hardwareMap.dcMotor.get("fR");
        DcMotor bR = hardwareMap.dcMotor.get("bR");

        // Reverse the right side so +power drives forward on all wheels
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ======== Mechanisms ========
        DcMotor intake  = hardwareMap.dcMotor.get("intake");   // rename to match your config
        DcMotor outtake = hardwareMap.dcMotor.get("outtake");  // rename to match your config

        // Sorting mechanism (positional servo)
        Servo sorter = hardwareMap.servo.get("sorter");

        // Initial states
        boolean intakeOn  = false;
        boolean outtakeOn = false;

        // Rising-edge detection
        boolean prevA = false;
        boolean prevB = false;
        boolean prevX = false;

        // sorter mode: 0=LEFT, 1=CENTER, 2=RIGHT
        int sorterMode = 1; // start centered
        sorter.setPosition(SORT_CENTER);

        // Make sure mechanisms are off at init
        intake.setPower(0);
        outtake.setPower(0);

        telemetry.addLine("Ready. A=intake toggle, B=outtake toggle, X=cycle sorter L-C-R");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ======== DRIVING (robot-centric mecanum) ========
            double y  = -gamepad1.left_stick_y;  // forward is -Y on stick
            double x  =  gamepad1.left_stick_x;  // strafe
            double rx =  gamepad1.right_stick_x; // rotate

            // Simple slow mode with left trigger
            double slow = 1.0 - (0.65 * gamepad1.left_trigger); // 1.0 -> 0.35
            double scale = DRIVE_POWER_SCALE * slow;

            // mecanum mixing
            double fl = (y + x + rx) * scale;
            double bl = (y - x + rx) * scale;
            double fr = (y - x - rx) * scale;
            double br = (y + x - rx) * scale;

            // normalize if any |power| > 1
            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(bl),
                                    Math.max(Math.abs(fr), Math.abs(br)))));

            fL.setPower(fl / max);
            bL.setPower(bl / max);
            fR.setPower(fr / max);
            bR.setPower(br / max);

            // ======== TOGGLES: Intake (A) & Outtake (B) ========
            boolean curA = gamepad1.a;
            boolean curB = gamepad1.b;

            if (curA && !prevA) {
                intakeOn = !intakeOn;
                intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
            }
            if (curB && !prevB) {
                outtakeOn = !outtakeOn;
                outtake.setPower(outtakeOn ? OUTTAKE_POWER : 0.0);
            }

            prevA = curA;
            prevB = curB;

            // ======== SORTING: X cycles L -> C -> R -> L ... ========
            boolean curX = gamepad1.x;
            if (curX && !prevX) {
                sorterMode = (sorterMode + 1) % 3;
                switch (sorterMode) {
                    case 0: sorter.setPosition(SORT_LEFT);   break;
                    case 1: sorter.setPosition(SORT_CENTER); break;
                    case 2: sorter.setPosition(SORT_RIGHT);  break;
                }
            }
            prevX = curX;

            // Quick overrides (useful for tuning)
            if (gamepad1.dpad_left)  sorter.setPosition(SORT_LEFT);
            if (gamepad1.dpad_up)    sorter.setPosition(SORT_CENTER);
            if (gamepad1.dpad_right) sorter.setPosition(SORT_RIGHT);

            // ======== TELEMETRY ========
            telemetry.addData("Drive scale", "%.2f", scale);
            telemetry.addData("Intake",  intakeOn ? "ON" : "OFF");
            telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
            telemetry.addData("Sorter mode", sorterMode == 0 ? "LEFT" : sorterMode == 1 ? "CENTER" : "RIGHT");
            telemetry.update();
        }
    }
}
