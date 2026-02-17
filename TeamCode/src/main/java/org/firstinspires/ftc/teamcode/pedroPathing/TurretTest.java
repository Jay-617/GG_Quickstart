package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "SNappyTurret CURRENT LOCKED + REV Color Auto")
public class TurretTest extends LinearOpMode {

    DcMotor LF, RF, LB, RB;
    DcMotor intake, intake2, spinner, outtake;
    Servo uppies, headc, Leftheadangle,Rightheadangle;
    IMU imu;

    RevColorSensorV3 cupColorSensor;

    static final int COUNTS_PER_REV = 8192;
    static final int CUP_COUNT = 3;
    static final int COUNTS_PER_CUP = COUNTS_PER_REV / CUP_COUNT;

    static final double SHIFT_DEGREES = 25;
    static final int SHIFT_TICKS = (int)(SHIFT_DEGREES / 360.0 * COUNTS_PER_REV);

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

    static final double HEADC_STOP = 0.5;
    static final double HEAD_SPEED = 0.35;
    static final double HEAD_RAMP = 0.05;

    double headcCmd = 0;
    double headcOutput = 0;

    boolean lastOuttake, outtakeOn;
    boolean lastUppies, uppiesOn;
    boolean headSideOpen = false;
    boolean lastHeadSide;


    boolean invertedControls;

    ElapsedTime buttonDelay = new ElapsedTime();

    boolean lastInvert, lastYawReset;
    boolean lastRightStick, lastLeftStick;
    boolean lastA, lastB, lastX, lastY;

    boolean justScanned = false;
    boolean buttonOverrideActive = false;

    String[] cupColors = new String[CUP_COUNT];

    @Override
    public void runOpMode() {

        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        intake = hardwareMap.dcMotor.get("intake");
        intake2 = hardwareMap.dcMotor.get("intake2");
        spinner = hardwareMap.dcMotor.get("spinner");
        outtake = hardwareMap.dcMotor.get("outtake");

        uppies = hardwareMap.servo.get("uppies");
        headc = hardwareMap.servo.get("headc");
        Leftheadangle = hardwareMap.servo.get("Leftheadangle");
        Rightheadangle = hardwareMap.servo.get("Rightheadangle");

        cupColorSensor = hardwareMap.get(RevColorSensorV3.class, "cupColor");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Rightheadangle.setDirection(Servo.Direction.REVERSE);

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        for (int i = 0; i < CUP_COUNT; i++) cupColors[i] = "empty";
        headc.setPosition(HEADC_STOP);

        waitForStart();

        while (opModeIsActive()) {

            int currentPos = spinner.getCurrentPosition();

            // merged mode: offset applied only for normal shifted mode
            int offset = 0;

            /* ===== DRIVE ===== */
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.b && !lastInvert && buttonDelay.seconds() > 0.3) {
                invertedControls = !invertedControls;
                buttonDelay.reset();
            }
            lastInvert = gamepad1.b;
            if (invertedControls) { lx = -lx; ly = -ly; }

            if (gamepad1.y && !lastYawReset) imu.resetYaw();
            lastYawReset = gamepad1.y;

            double yaw = getYaw();
            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);

            double tempX = lx * cosA - ly * sinA;
            double tempY = lx * sinA + ly * cosA;

            double fl = tempY + tempX + rx;
            double bl = tempY - tempX + rx;
            double fr = tempY - tempX - rx;
            double br = tempY + tempX - rx;

            double speed = 0.8 + (0.2 * gamepad1.left_trigger) - (0.6 * gamepad1.right_trigger);
            setDrive(fl * speed, bl * speed, fr * speed, br * speed);

            /* ===== MANUAL STEP ===== */
            if (gamepad2.a && !lastA) {
                currentCup = (currentCup + 1) % CUP_COUNT;
                spinnerTarget = getShortestTarget(currentPos, currentCup, offset);
                integral = 0;
            }
            lastA = gamepad2.a;

            if (gamepad2.b && !lastB) {
                currentCup = (currentCup - 1 + CUP_COUNT) % CUP_COUNT;
                spinnerTarget = getShortestTarget(currentPos, currentCup, offset);
                integral = 0;
            }
            lastB = gamepad2.b;

            /* ===== BUTTON OVERRIDE ===== */
            if (gamepad2.x && !lastX) {
                moveClosestColorToShiftedTick("purple", 2730);
            }
            lastX = gamepad2.x;

            if (gamepad2.y && !lastY) {
                moveClosestColorToShiftedTick("green", 2730);
            }
            lastY = gamepad2.y;

            /* ===== CUP RELEASE LOGIC ===== */
            if (buttonOverrideActive && Math.abs(spinnerTarget - currentPos) <= DEADZONE) {

                // Stop the spinner
                spinner.setPower(0);

                // Move uppies up
                uppies.setPosition(0.6); // adjust as needed for “up” position
                sleep(300); // wait 0.3 seconds for it to fully move

                // Move uppies back down
                uppies.setPosition(0);
                sleep(300); // wait for down movement

                // Mark the cup at 2730 as empty
                for (int i = 0; i < CUP_COUNT; i++) {
                    int cupPos = i * COUNTS_PER_CUP;
                    int diff = cupPos - 2730;
                    if (diff > COUNTS_PER_REV / 2) diff -= COUNTS_PER_REV;
                    if (diff < -COUNTS_PER_REV / 2) diff += COUNTS_PER_REV;

                    if (Math.abs(diff) <= DEADZONE) {
                        cupColors[i] = "empty";
                        break;
                    }
                }

                // Finish the override sequence
                buttonOverrideActive = false;
            }

            /* ===== PID ===== */
            int error = spinnerTarget - currentPos;

            if (Math.abs(error) < 200) integral += error;
            else integral = 0;

            double output = (kP * error) + (kI * integral) + (kD * (error - lastError));
            if (Math.abs(error) < SLOW_ZONE) output *= 0.5;
            if (Math.abs(error) <= DEADZONE) output = Math.signum(error) * HOLD_POWER;

            output = Math.max(-0.25, Math.min(0.25, output));
            spinner.setPower(output);
            lastError = error;

            /* ===== INTAKE ===== */
            double intakePower = 0;
            if (Math.abs(error) > 100) intakePower = 0.3;
            if (gamepad2.right_trigger > 0.1) intakePower = 0.8;
            else if (gamepad2.left_trigger > 0.1) intakePower = -0.8;

            intake.setPower(intakePower);
            intake2.setPower(intakePower);

            /* ===== OUTTAKE ===== */
            if (gamepad2.left_bumper && !lastOuttake) outtakeOn = !outtakeOn;
            lastOuttake = gamepad2.left_bumper;
            outtake.setPower(outtakeOn ? 1 : 0);

            /* ===== UPPIES ===== */
            if (gamepad2.dpad_up && !lastUppies) uppiesOn = !uppiesOn;
            lastUppies = gamepad2.dpad_up;
            uppies.setPosition(uppiesOn ? 0.6 : 0);

            /* ===== HEADSIDE Servos ===== */
            if (gamepad2.right_bumper && !lastHeadSide) {
                headSideOpen = !headSideOpen;
            }
            lastHeadSide = gamepad2.right_bumper;

            if (headSideOpen) {
                Leftheadangle.setPosition(0.10);
                Rightheadangle.setPosition(0.86);
            } else {
                Leftheadangle.setPosition(0.00);
                Rightheadangle.setPosition(0.76);
            }


            /* ===== HEAD ===== */
            if (gamepad2.dpad_left) headcCmd = -HEAD_SPEED;
            else if (gamepad2.dpad_right) headcCmd = HEAD_SPEED;
            else headcCmd = 0;

            headcOutput += (headcCmd - headcOutput) * HEAD_RAMP;
            headc.setPosition(HEADC_STOP + headcOutput);

            /* ===== AUTO SCAN ===== */
            if (!buttonOverrideActive &&
                    Math.abs(output) <= HOLD_POWER &&
                    !justScanned) {

                int r = cupColorSensor.red();
                int g = cupColorSensor.green();
                int b = cupColorSensor.blue();

                if (g > r && g > b && g > 80) cupColors[currentCup] = "green";
                else if (b > r && b > g && b > 80) cupColors[currentCup] = "purple";
                else cupColors[currentCup] = "empty";

                justScanned = true;

                if (!"empty".equals(cupColors[currentCup])) {
                    advanceToNextEmptyCup();
                }
            }

            if (Math.abs(output) > HOLD_POWER) justScanned = false;

            telemetry.addData("Current Cup", currentCup + 1);
            telemetry.addData("Spinner Pos", currentPos);
            telemetry.addData("Override", buttonOverrideActive);
            for (int i = 0; i < CUP_COUNT; i++)
                telemetry.addData("Cup " + (i + 1), cupColors[i]);
            telemetry.update();
        }
    }

    /* ===== MERGED METHOD ===== */
    private void moveClosestColorToShiftedTick(String color, int targetTick) {

        int currentPos = spinner.getCurrentPosition();
        int bestCup = -1;
        int bestDist = Integer.MAX_VALUE;

        for (int i = 0; i < CUP_COUNT; i++) {
            if (!color.equals(cupColors[i])) continue;

            int cupPos = i * COUNTS_PER_CUP; // no offset
            int diff = cupPos - targetTick;

            if (diff > COUNTS_PER_REV / 2) diff -= COUNTS_PER_REV;
            if (diff < -COUNTS_PER_REV / 2) diff += COUNTS_PER_REV;

            int dist = Math.abs(diff);
            if (dist < bestDist) {
                bestDist = dist;
                bestCup = i;
            }
        }

        if (bestCup != -1) {
            currentCup = bestCup;
            spinnerTarget = getShortestTarget(currentPos, bestCup, targetTick);
            integral = 0;
            lastError = 0;
            buttonOverrideActive = true;
        }
    }

    private void advanceToNextEmptyCup() {
        for (int i = 1; i <= CUP_COUNT; i++) {
            int next = (currentCup + i) % CUP_COUNT;
            if ("empty".equals(cupColors[next])) {
                currentCup = next;
                spinnerTarget = getShortestTarget(
                        spinner.getCurrentPosition(),
                        currentCup,
                        0
                );
                integral = 0;
                return;
            }
        }
    }

    private int getShortestTarget(int current, int cup, int offset) {
        int target = cup * COUNTS_PER_CUP + offset;
        int diff = target - current;
        if (diff > COUNTS_PER_REV / 2) diff -= COUNTS_PER_REV;
        if (diff < -COUNTS_PER_REV / 2) diff += COUNTS_PER_REV;
        return current + diff;
    }

    private double getYaw() {
        YawPitchRollAngles a = imu.getRobotYawPitchRollAngles();
        return a.getYaw(AngleUnit.RADIANS);
    }

    private void setDrive(double fl, double bl, double fr, double br) {
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }
        LF.setPower(fl);
        LB.setPower(bl);
        RF.setPower(fr);
        RB.setPower(br);
    }
}
