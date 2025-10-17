//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.CRServo;
//
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@TeleOp(name = "TeleOp (Field-Centric, Dual Outtake 315RPM, CR Sorter)", group = "A1")
//public class testingOT extends LinearOpMode {
//
//    // === Driver feel / controls ===
//    public static double DRIVE_POWER_SCALE = 1.0;
//    public static double SLOW_FACTOR       = 0.65; // LT reduces to ~35%
//    public static double DEADZONE          = 0.05;
//    public static double EXPO              = 2.0;  // 2.0 = squaring
//    public static boolean HEADING_HOLD_ENABLED = true;
//    public static double HEADING_HOLD_KP   = 1.0;
//
//    // === Mechanisms ===
//    public static double INTAKE_POWER      = 1.0;   // A toggles intake motor power on/off
//    public static double OUTTAKE_RPM       = 315.0; // B toggles both outtake motors at this RPM
//    // Common goBILDA 312RPM encoder ~537.7 ticks/rev. Adjust to your motor encoder spec.
//    public static double OUTTAKE_TICKS_PER_REV = 537.7;
//
//    // Continuous-rotation sorter servo: X toggles at fixed speed
//    public static double SORTER_SPEED      = 1.0;   // 0..1 speed; reverse by negative value
//
//    // Internal
//    private double holdHeadingRad = 0.0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // ======== Drivetrain (4 DC motors) ========
//        DcMotor FL = hardwareMap.dcMotor.get("FL");
//        DcMotor BL = hardwareMap.dcMotor.get("BL");
//        DcMotor FR = hardwareMap.dcMotor.get("FR");
//        DcMotor BR = hardwareMap.dcMotor.get("BR");
//        /// //
//        // Dual outtake (DcMotorEx for velocity control). Rename to match your config.
//        DcMotorEx outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
//        DcMotorEx outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");
//        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
///// //////////
//
//
//        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Reverse right side so +power drives forward on all wheels
//        FR.setDirection(DcMotorSimple.Direction.REVERSE);
//        BR.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // ======== Mechanisms ========
//        DcMotor intake = hardwareMap.dcMotor.get("intake"); // rename to match your config
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        // If your mechanism requires opposite spin directions, reverse one side here:
//        // outtakeR.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // CR sorter servo (speed-based)
//        CRServo sorter = hardwareMap.get(CRServo.class, "sorter");
//
//        // ======== IMU Setup (FIELD-CENTRIC) ========
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot orientationOnRobot =
//                new RevHubOrientationOnRobot(LogoFacingDirection.DOWN, UsbFacingDirection.LEFT);
//        IMU.Parameters imuParams = new IMU.Parameters(orientationOnRobot);
//        imu.initialize(imuParams);
//        imu.resetYaw();
//        holdHeadingRad = 0.0;
//
//        // ======== State ========
//        boolean intakeOn   = false;
//        boolean outtakeOn  = false; // B toggles both outtakes at 315 RPM
//        boolean sorterOn   = false; // X toggles CR servo at SORTER_SPEED
//
//        // edge detection
//        boolean prevA=false, prevB=false, prevX=false, prevY=false;
//
//        // Initialize mechanisms off
//        intake.setPower(0);
//        outtakeL.setPower(0);
//        outtakeR.setPower(0);
//        sorter.setPower(0);
//
//        telemetry.addLine("A=intake toggle | B=outtake 315RPM toggle | X=sorter toggle | Y=reset heading");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            // ======== DRIVING (FIELD-CENTRIC MECANUM) ========
//            double y  = -gamepad1.left_stick_y;  // forward
//            double x  =  gamepad1.left_stick_x;  // strafe
//            double rx =  gamepad1.right_stick_x; // rotate
//
//            // Deadband + expo
//            y  = expo(deadband(y,  DEADZONE), EXPO);
//            x  = expo(deadband(x,  DEADZONE), EXPO);
//            rx = expo(deadband(rx, DEADZONE), EXPO);
//
//            // Slow mode
//            double slow  = 1.0 - (SLOW_FACTOR * gamepad1.left_trigger); // 1.0 -> 0.35
//            double scale = DRIVE_POWER_SCALE * slow;
//
//            // Field-centric rotate
//            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double botHeading = -yaw;
//            double cos = Math.cos(botHeading), sin = Math.sin(botHeading);
//            double rotX = x * cos - y * sin;
//            double rotY = x * sin + y * cos;
//
//            // Heading hold
//            double rxCmd = rx;
//            if (HEADING_HOLD_ENABLED) {
//                if (Math.abs(rx) > 0.02) {
//                    holdHeadingRad = -yaw;
//                } else {
//                    double current = -yaw;
//                    double err = normalizeRadians(holdHeadingRad - current);
//                    rxCmd += HEADING_HOLD_KP * err;
//                }
//            }
//
//            // Mecanum mix
//            double fl = (rotY + rotX + rxCmd) * scale;
//            double bl = (rotY - rotX + rxCmd) * scale;
//            double fr = (rotY - rotX - rxCmd) * scale;
//            double br = (rotY + rotX - rxCmd) * scale;
//
//            // Normalize
//            double max = Math.max(1.0,
//                    Math.max(Math.abs(fl),
//                            Math.max(Math.abs(bl),
//                                    Math.max(Math.abs(fr), Math.abs(br)))));
//            FL.setPower(fl / max);
//            BL.setPower(bl / max);
//            FR.setPower(fr / max);
//            BR.setPower(br / max);
//
//            // ======== HEADING RESET (Y) ========
//            boolean curY = gamepad1.y;
//            if (curY && !prevY) {
//                imu.resetYaw();
//                holdHeadingRad = 0.0;
//            }
//            prevY = curY;
//
//            // ======== INTAKE TOGGLE (A) ========
//            boolean curA = gamepad1.a;
//            if (curA && !prevA) {
//                intakeOn = !intakeOn;
//                intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
//            }
//            prevA = curA;
//
//            // ======== OUTTAKE TOGGLE (B) → both motors @ 315 RPM ========
//<<<<<<< Updated upstream
//            boolean curB = gamepad1.b;
//            if (curB && !prevB) {
//                outtakeOn = !outtakeOn;
//                if (outtakeOn) {
//                    outtakeL.setPower(0.5);
//                    outtakeR.setPower(0.5);
////                    double ticksPerSec = (OUTTAKE_RPM * OUTTAKE_TICKS_PER_REV) / 60.0;
////                    outtakeL.setVelocity(ticksPerSec);
////                    outtakeR.setVelocity(ticksPerSec);
//                } else {
//                    outtakeL.setPower(0.0);
//                    outtakeR.setPower(0.0);
//                }
//=======
////            boolean curB = gamepad1.b;
////            if (curB && !prevB) {
////                outtakeOn = !outtakeOn;
////                if (outtakeOn) {
////                    outtakeL.setVelocity();
//////                    double ticksPerSec = (OUTTAKE_RPM * OUTTAKE_TICKS_PER_REV) / 60.0;
//////                    outtakeL.setVelocity(ticksPerSec);
//////                    outtakeR.setVelocity(ticksPerSec);
////                } else {
////                    outtakeL.setPower(0.0);
////                    outtakeR.setPower(0.0);
////                }
////            }
////            prevB = curB;
//
//            // Inside your loop() or update method:
//            if (gamepad1.b) {
//                outtakeL.setPower(1);
//                outtakeR.setPower(1);
//            } else {
//                outtakeL.setPower(0);
//                outtakeR.setPower(0);
//>>>>>>> Stashed changes
//            }
//
//            // ======== SORTER TOGGLE (X) → CR servo @ fixed speed ========
//            boolean curX = gamepad1.x;
//            if (curX && !prevX) {
//                sorterOn = !sorterOn;
//                sorter.setPower(sorterOn ? SORTER_SPEED : 0.0);
//            }
//            prevX = curX;
//
//            // ======== TELEMETRY ========
//            telemetry.addData("Heading (deg)", "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("Drive scale", "%.2f", scale);
//            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
//            telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
////            if (outtakeOn) {
////                telemetry.addData("OutL vel (tps)", "%.0f", outtakeL.getVelocity());
////                telemetry.addData("OutR vel (tps)", "%.0f", outtakeR.getVelocity());
////            }
//            if (sorterOn) {
//                telemetry.addData("Sorter", "ON @ %.2f", SORTER_SPEED);
//            } else {
//                telemetry.addData("Sorter", "OFF");
//            }
//            telemetry.update();
//        }
//    }
//
//    // ======== Helpers ========
//    private static double deadband(double v, double dz) {
//        return (Math.abs(v) < dz) ? 0.0 : v;
//    }
//    private static double expo(double v, double exp) {
//        exp = Math.max(1.0, exp);
//        return Math.copySign(Math.pow(Math.abs(v), exp), v);
//    }
//    private static double normalizeRadians(double a) {
//        while (a > Math.PI)  a -= 2*Math.PI;
//        while (a < -Math.PI) a += 2*Math.PI;
//        return a;
//    }
//}
//
