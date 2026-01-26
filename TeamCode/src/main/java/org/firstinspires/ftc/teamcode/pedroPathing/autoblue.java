package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "blue auto")
public class autoblue extends LinearOpMode {


    /* ===== HARDWARE ===== */
    DcMotor LF, RF, LB, RB;
    DcMotor spinner, intake2, outtake, outtake2;
    Servo uppies;
    IMU imu;
    RevColorSensorV3 cupColorSensor;


    /* ===== DRIVE CONSTANTS ===== */
    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH =
            COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);


    /* ===== HEADING HOLD ===== */
    double targetHeading = 0;
    static final double HEADING_kP = 1.8;


    /* ===== SPINNER CONSTANTS ===== */
    static final int COUNTS_PER_REV = 8192;
    static final int CUP_COUNT = 3;
    static final int COUNTS_PER_CUP = COUNTS_PER_REV / CUP_COUNT;


    /* ===== TELEOP PID ===== */
    static final double kP = 0.0026;
    static final double kI = 0.000015;
    static final double kD = 0.00009;
    static final double HOLD_POWER = 0.08;
    static final int DEADZONE = 4;
    static final int SLOW_ZONE = 220;


    int currentCup = 0;
    int spinnerTarget = 0;
    double integral = 0;
    double lastError = 0;


    @Override
    public void runOpMode() {


        /* ===== MAP HARDWARE ===== */
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");


        spinner = hardwareMap.dcMotor.get("spinner");
        intake2 = hardwareMap.dcMotor.get("intake2");
        outtake = hardwareMap.dcMotor.get("outtake");
        outtake2 = hardwareMap.dcMotor.get("outtake2");


        uppies = hardwareMap.servo.get("uppies");
        cupColorSensor = hardwareMap.get(RevColorSensorV3.class, "cupColor");
        imu = hardwareMap.get(IMU.class, "imu");


        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);


        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);


        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));


        telemetry.addLine("READY");
        telemetry.update();


        waitForStart();
        imu.resetYaw();


        targetHeading = getYaw();


        /* ===== DRIVE BACK ===== */
        driveBackward(45, 1.0);


        /* ===== OUTTAKE ON ===== */
        outtake.setPower(1);
        outtake2.setPower(1);


        int shootTick = 2730;


        for (int i = 0; i < 4 && opModeIsActive(); i++) {
            autoShootCup(i, shootTick);
            sleep(500);
        }


        outtake.setPower(0);
        outtake2.setPower(0);


//        /* ===== TURN BEFORE STRAFE ===== */
//        turnToHeading(Math.toRadians(-90), 0.6);


        /* ===== STRAFE RIGHT ===== */
        strafeLeft(30, 1.0);
    }


    /* ===== TURN TO HEADING ===== */
    private void turnToHeading(double targetRadians, double maxPower) {


        while (opModeIsActive()) {
            double error = targetRadians - getYaw();


            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;


            if (Math.abs(error) < Math.toRadians(1.5)) break;


            double turnPower = error * HEADING_kP;
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));


            setDrive(-turnPower, -turnPower, turnPower, turnPower);
            updateSpinnerPID();
        }


        setDrive(0, 0, 0, 0);
        targetHeading = targetRadians;
    }


    /* ===== AUTO SHOOT ===== */
    private void autoShootCup(int cupIndex, int targetTick) {


        int currentPos = spinner.getCurrentPosition();
        currentCup = cupIndex;


        spinnerTarget = getShortestTarget(currentPos, cupIndex, targetTick);
        integral = 0;
        lastError = 0;


        while (opModeIsActive()) {
            updateSpinnerPID();
            if (Math.abs(spinnerTarget - spinner.getCurrentPosition()) <= DEADZONE) break;
        }


        spinner.setPower(0);
        sleep(120);


        uppies.setPosition(0.6);
        sleep(300);
        uppies.setPosition(0);
        sleep(300);
    }


    /* ===== SPINNER PID ===== */
    private void updateSpinnerPID() {


        int error = spinnerTarget - spinner.getCurrentPosition();


        if (Math.abs(error) < 200) integral += error;
        else integral = 0;


        double output =
                (kP * error) +
                        (kI * integral) +
                        (kD * (error - lastError));


        if (Math.abs(error) < SLOW_ZONE) output *= 0.5;
        if (Math.abs(error) <= DEADZONE)
            output = Math.signum(error) * HOLD_POWER;


        output = Math.max(-0.25, Math.min(0.25, output));
        spinner.setPower(output);


        intake2.setPower(Math.abs(error) > 100 ? 0.3 : 0);
        lastError = error;
    }


    /* ===== DRIVE HELPERS ===== */
    private void driveBackward(double inches, double power) {
        encoderDrive(-inches, -inches, -inches, -inches, power);
    }


    private void strafeRight(double inches, double power) {
        encoderDrive(inches, -inches, -inches, inches, power);
    }
    private void strafeLeft(double inches, double power) {
        encoderDrive(-inches, inches, inches, -inches, power);
    }

    private void encoderDrive(double flIn, double blIn,
                              double frIn, double brIn,
                              double power) {


        int flTarget = LF.getCurrentPosition() + (int)(flIn * COUNTS_PER_INCH);
        int blTarget = LB.getCurrentPosition() + (int)(blIn * COUNTS_PER_INCH);
        int frTarget = RF.getCurrentPosition() + (int)(frIn * COUNTS_PER_INCH);
        int brTarget = RB.getCurrentPosition() + (int)(brIn * COUNTS_PER_INCH);


        LF.setTargetPosition(flTarget);
        LB.setTargetPosition(blTarget);
        RF.setTargetPosition(frTarget);
        RB.setTargetPosition(brTarget);


        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive() &&
                (LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy())) {


            double correction = getHeadingCorrection();
            double left = power - correction;
            double right = power + correction;


            setDrive(left, left, right, right);
            updateSpinnerPID();
        }


        setDrive(0, 0, 0, 0);


        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private double getHeadingCorrection() {
        double error = targetHeading - getYaw();
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error * HEADING_kP;
    }


    private void setDrive(double fl, double bl, double fr, double br) {
        LF.setPower(fl);
        LB.setPower(bl);
        RF.setPower(fr);
        RB.setPower(br);
    }


    private int getShortestTarget(int current, int cup, int offset) {
        int target = cup * COUNTS_PER_CUP + offset;
        int diff = target - current;
        if (diff > COUNTS_PER_REV / 2) diff -= COUNTS_PER_REV;
        if (diff < -COUNTS_PER_REV / 2) diff += COUNTS_PER_REV;
        return current + diff;
    }


    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}

