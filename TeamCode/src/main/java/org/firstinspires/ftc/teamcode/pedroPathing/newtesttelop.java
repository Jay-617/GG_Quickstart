//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//@TeleOp(name = "SNappy teleop latest")
//public class newtesttelop extends LinearOpMode {
//
//    DcMotor LF, RF, LB, RB;
//    DcMotorEx outtake, outtake2;
//    DcMotor intake2, spinner;
//    Servo uppies, headc, Leftheadangle, Rightheadangle;
//    IMU imu;
//
//    RevColorSensorV3 cupColorSensor;
//
//    static final int COUNTS_PER_REV = 8192;
//    static final int CUP_COUNT = 3;
//    static final int COUNTS_PER_CUP = COUNTS_PER_REV / CUP_COUNT;
//
//    static final double SHIFT_DEGREES = 25;
//    static final int SHIFT_TICKS = (int)(SHIFT_DEGREES / 360.0 * COUNTS_PER_REV);
//
//    static final double kP = 0.0026;
//    static final double kI = 0.000015;
//    static final double kD = 0.00009;
//    static final double HOLD_POWER = 0.08;
//    static final int DEADZONE = 4;
//    static final int SLOW_ZONE = 220;
//
//    int currentCup = 0;
//    int spinnerTarget = 0;
//
//    double integral = 0;
//    double lastError = 0;
//
//    static final double HEADC_STOP = 0.5;
//    static final double HEAD_SPEED = 0.35;
//    static final double HEAD_RAMP = 0.05;
//
//    double headcCmd = 0;
//    double headcOutput = 0;
//
//    boolean lastOuttake, outtakeOn;
//    boolean headSideOpen = false;
//    boolean lastHeadSide;
//
//    boolean shootMode = false;
//    boolean invertedControls;
//
//    ElapsedTime buttonDelay = new ElapsedTime();
//
//    boolean lastInvert, lastYawReset;
//    boolean lastRightStick, lastLeftStick;
//    boolean lastA, lastB, lastX, lastY;
//
//    boolean justScanned = false;
//    boolean buttonOverrideActive = false;
//
//    String[] cupColors = new String[CUP_COUNT];
//
//    /* ===== OUTTAKE PIDF SETTINGS ===== */
//    static final double OUTTAKE_VELOCITY = 3000;
//    static final double OUTTAKE_P = 0.0;
//    static final double OUTTAKE_I = 0.0;
//    static final double OUTTAKE_D = 0.0;
//    static final double OUTTAKE_F = 12.0;
//
//    /* ===== UPPIES AUTO SETTINGS ===== */
//    static final double UPPIES_UP_POS = 0.35;
//    static final double UPPIES_DOWN_POS = 0.0;
//    static final double UPPIES_HOLD_TIME = 0.5; // seconds
//
//    boolean uppiesBusy = false;
//    boolean uppiesUp = false;
//    boolean lastUppies;
//    ElapsedTime uppiesTimer = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//
//        // ===== HARDWARE INIT =====
//        LF = hardwareMap.dcMotor.get("FL");
//        LB = hardwareMap.dcMotor.get("BL");
//        RF = hardwareMap.dcMotor.get("FR");
//        RB = hardwareMap.dcMotor.get("BR");
//
//        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
//        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
//
//        intake2 = hardwareMap.dcMotor.get("intake2");
//        spinner = hardwareMap.dcMotor.get("spinner");
//
//        uppies = hardwareMap.servo.get("uppies");
//        headc = hardwareMap.servo.get("headc");
//        Leftheadangle = hardwareMap.servo.get("Leftheadangle");
//        Rightheadangle = hardwareMap.servo.get("Rightheadangle");
//
//        cupColorSensor = hardwareMap.get(RevColorSensorV3.class, "cupColor");
//
//        LF.setDirection(DcMotorSimple.Direction.FORWARD);
//        LB.setDirection(DcMotorSimple.Direction.REVERSE);
//        RF.setDirection(DcMotorSimple.Direction.FORWARD);
//        RB.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
//        Rightheadangle.setDirection(Servo.Direction.REVERSE);
//
//        /* ===== OUTTAKE PIDF INIT ===== */
//        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        PIDFCoefficients pidf = new PIDFCoefficients(
//                OUTTAKE_P, OUTTAKE_I, OUTTAKE_D, OUTTAKE_F);
//        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//
//        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
//
//        for (int i = 0; i < CUP_COUNT; i++) cupColors[i] = "empty";
//
//        uppies.setPosition(UPPIES_DOWN_POS);
//        headc.setPosition(HEADC_STOP);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            int currentPos = spinner.getCurrentPosition();
//            runCupReleaseAt2730(currentPos);
//
//            /* ===== DRIVE ===== */
//            double lx = gamepad1.left_stick_x;
//            double ly = -gamepad1.left_stick_y;
//            double rx = gamepad1.right_stick_x;
//
//            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
//                invertedControls = !invertedControls;
//                buttonDelay.reset();
//            }
//            lastInvert = gamepad1.b;
//
//            if (invertedControls) {
//                lx = -lx;
//                ly = -ly;
//            }
//
//            if (gamepad1.y && !lastYawReset) imu.resetYaw();
//            lastYawReset = gamepad1.y;
//
//            double yaw = getYaw();
//            double cosA = Math.cos(-yaw);
//            double sinA = Math.sin(-yaw);
//
//            double rotX = lx * cosA - ly * sinA;
//            double rotY = lx * sinA + ly * cosA;
//
//            double fl = rotY + rotX + rx;
//            double bl = rotY - rotX + rx;
//            double fr = rotY - rotX - rx;
//            double br = rotY + rotX - rx;
//
//            double speed = 1 - (0.7 * gamepad1.right_trigger);
//            setDrive(fl * speed, bl * speed, fr * speed, br * speed);
//
//            /* ===== SHOOT MODE + SPINNER + INTAKE ===== */
//            // <<< keep all your original code for shooter, spinner, intake, head, etc. here >>>
//
//            /* ===== UPPIES AUTO SEQUENCE ===== */
//            if (gamepad2.dpad_up && !lastUppies && !uppiesBusy) {
//                uppiesBusy = true;
//                uppiesUp = true;
//                uppiesTimer.reset();
//                uppies.setPosition(UPPIES_UP_POS);
//            }
//            lastUppies = gamepad2.dpad_up;
//
//            if (uppiesBusy && uppiesUp && uppiesTimer.seconds() > UPPIES_HOLD_TIME) {
//                uppiesUp = false;
//                uppies.setPosition(UPPIES_DOWN_POS);
//            }
//
//            if (uppiesBusy && !uppiesUp && uppiesTimer.seconds() > UPPIES_HOLD_TIME + 0.2) {
//                uppiesBusy = false;
//            }
//
//            /* ===== TELEMETRY ===== */
//            telemetry.addData("Uppies", uppiesUp ? "✅ UP" : "❌ DOWN");
//            // <<< keep all your other telemetry here >>>
//            telemetry.update();
//        }
//    }
//
//    /* ===== METHODS ===== */
//    private void runCupReleaseAt2730(int currentPos) {
//        if (!buttonOverrideActive) return;
//        if (Math.abs(spinnerTarget - currentPos) > DEADZONE) return;
//        spinner.setPower(0);
//        uppies.setPosition(0.6);
//        sleep(300);
//        uppies.setPosition(0);
//        sleep(300);
//        cupColors[currentCup] = "empty";
//        buttonOverrideActive = false;
//    }
//
//    private void advanceToNextEmptyCup() {
//        for (int i = 1; i <= CUP_COUNT; i++) {
//            int next = (currentCup + i) % CUP_COUNT;
//            if ("empty".equals(cupColors[next])) {
//                currentCup = next;
//                spinnerTarget = getShortestTarget(spinner.getCurrentPosition(), currentCup, 0);
//                integral = 0;
//                return;
//            }
//        }
//    }
//
//    private int getShortestTarget(int current, int cup, int offset) {
//        int target = cup * COUNTS_PER_CUP + offset;
//        int diff = target - current;
//        if (diff > COUNTS_PER_REV / 2) diff -= COUNTS_PER_REV;
//        if (diff < -COUNTS_PER_REV / 2) diff += COUNTS_PER_REV;
//        return current + diff;
//    }
//
//    private double getYaw() {
//        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
//        return a.getYaw(AngleUnit.RADIANS);
//    }
//
//    private void setDrive(double fl, double bl, double fr, double br) {
//        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
//        if (max > 1.0) { fl /= max; bl /= max; fr /= max; br /= max; }
//        LF.setPower(fl);
//        LB.setPower(bl);
//        RF.setPower(fr);
//        RB.setPower(br);
//    }
//}
