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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "SNappy Auto Full Scan")
public class SnappyAutoFullScan extends LinearOpMode {

    /* ===== DRIVE ===== */
    DcMotor LF, RF, LB, RB;

    /* ===== MECHANISMS ===== */
    DcMotor spinner, intake2,outtake,outtake2;
    Servo uppies;
    IMU imu;
    RevColorSensorV3 cupColorSensor;

    /* ===== DRIVE MATH ===== */
    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    /* ===== SPINNER CONSTANTS ===== */
    static final int COUNTS_PER_REV = 8192;
    static final int CUP_COUNT = 3;
    static final int COUNTS_PER_CUP = COUNTS_PER_REV / CUP_COUNT;

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

    boolean buttonOverrideActive = false;
    boolean justScanned = false;

    String[] cupColors = new String[CUP_COUNT];

    @Override
    public void runOpMode() {

        /* ===== HARDWARE MAP ===== */
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        spinner = hardwareMap.dcMotor.get("spinner");
        intake2 = hardwareMap.dcMotor.get("intake2"); // intake motor
        outtake = hardwareMap.dcMotor.get("outtake");
        outtake2 = hardwareMap.dcMotor.get("outtake2");
        uppies = hardwareMap.servo.get("uppies");
        cupColorSensor = hardwareMap.get(RevColorSensorV3.class, "cupColor");
        imu = hardwareMap.get(IMU.class, "imu");

        /* ===== MOTOR SETUP ===== */
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);  // if needed
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD); // or REVERSE depending on wiring
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        for (int i = 0; i < CUP_COUNT; i++) cupColors[i] = "empty";

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        imu.resetYaw();

        // Scan all cups: spinner moves, intake runs, scan after reaching position
        while (opModeIsActive() && !allCupsScanned()) {

            advanceToNextEmptyCup();                // pick next cup
            spinnerTarget = currentCup * COUNTS_PER_CUP; // set spinner position

            // Move spinner + intake until it reaches the cup
            while(opModeIsActive() && !spinnerAtTarget()) {
                updateSpinnerPID();  // moves spinner + intake
                sendTelemetry();
            }

            scanCurrentCup();  // scan color only after spinner settled
            sendTelemetry();
            sleep(100); // small pause to stabilize sensor
        }

        // Drive backward 30 inches at 0.5 speed
        driveBackward(10, 0.5);
        setOuttakes(.1);
        sleep(2000);
        setOuttakes(0);
        sendTelemetry();
    }

    /* ===== DRIVE METHODS ===== */
    private void driveForward(double inches, double power) { encoderDrive(inches, inches, inches, inches, power); }
    private void driveBackward(double inches, double power) { encoderDrive(-inches, -inches, -inches, -inches, power); }
    private void strafeLeft(double inches, double power) { encoderDrive(-inches, inches, inches, -inches, power); }
    private void strafeRight(double inches, double power) { encoderDrive(inches, -inches, -inches, inches, power); }

    /* ===== TURN METHOD ===== */
    private void turnDegrees(double targetDegrees, double power) {
        double startYaw = getYaw();
        double targetYaw = normalizeRadians(startYaw + Math.toRadians(targetDegrees));
        if (targetDegrees > 0) setDrive(power, power, -power, -power);
        else setDrive(-power, -power, power, power);

        while (opModeIsActive()) {
            double currentYaw = getYaw();
            if (Math.abs(normalizeRadians(targetYaw - currentYaw)) < Math.toRadians(1)) break;
            sendTelemetry();
        }
        setDrive(0, 0, 0, 0);
    }
    private void setOuttakes(double power) {
        outtake.setPower(power);
        outtake2.setPower(power);
    }


    private double getYaw() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }
    private double normalizeRadians(double angle) { while(angle>Math.PI) angle-=2*Math.PI; while(angle<-Math.PI) angle+=2*Math.PI; return angle; }

    private void encoderDrive(double flIn, double blIn, double frIn, double brIn, double power) {
        int flTarget = LF.getCurrentPosition() + (int)(flIn*COUNTS_PER_INCH);
        int blTarget = LB.getCurrentPosition() + (int)(blIn*COUNTS_PER_INCH);
        int frTarget = RF.getCurrentPosition() + (int)(frIn*COUNTS_PER_INCH);
        int brTarget = RB.getCurrentPosition() + (int)(brIn*COUNTS_PER_INCH);

        LF.setTargetPosition(flTarget); LB.setTargetPosition(blTarget);
        RF.setTargetPosition(frTarget); RB.setTargetPosition(brTarget);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrive(power, power, power, power);

        while(opModeIsActive() && (LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy())) {
            updateSpinnerPID(); // spinner + intake logic during movement
            sendTelemetry();
        }

        setDrive(0,0,0,0);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDrive(double fl, double bl, double fr, double br) { LF.setPower(fl); LB.setPower(bl); RF.setPower(fr); RB.setPower(br); }

    /* ===== SPINNER + INTAKE LOGIC ===== */
    private void updateSpinnerPID() {
        int currentPos = spinner.getCurrentPosition();
        int error = spinnerTarget - currentPos;

        if (Math.abs(error) < 200) integral += error; else integral = 0;
        double output = (kP*error) + (kI*integral) + (kD*(error-lastError));
        if(Math.abs(error)<SLOW_ZONE) output*=0.5;
        if(Math.abs(error)<=DEADZONE) output = Math.signum(error)*HOLD_POWER;
        output = Math.max(-0.25, Math.min(0.25, output));
        spinner.setPower(output);

        // Intake runs while spinner moving
        intake2.setPower(Math.abs(error) > 100 ? 0.3 : 0);

        lastError = error;
        runCupReleaseAt2730(currentPos);
    }

    private void runCupReleaseAt2730(int currentPos) {
        if(!buttonOverrideActive) return;
        if(Math.abs(spinnerTarget-currentPos)>DEADZONE) return;
        spinner.setPower(0);
        intake2.setPower(0); // stop intake too
        uppies.setPosition(0.6);
        sleep(300);
        uppies.setPosition(0);
        sleep(300);
        cupColors[currentCup]="empty";
        buttonOverrideActive=false;
    }

    private void scanCurrentCup() {
        if(justScanned) return;
        int greenCount=0, purpleCount=0;
        for(int i=0;i<5;i++){
            int r=cupColorSensor.red();
            int g=cupColorSensor.green();
            int b=cupColorSensor.blue();
            if(g>r && g>b && g>50) greenCount++;
            else if(b>r && b>g && b>50) purpleCount++;
            sleep(50);
        }
        if(greenCount>purpleCount && greenCount>2) cupColors[currentCup]="green";
        else if(purpleCount>greenCount && purpleCount>2) cupColors[currentCup]="purple";
        else cupColors[currentCup]="empty";
        justScanned=true;
    }

    private void advanceToNextEmptyCup() {
        for(int i=1;i<=CUP_COUNT;i++){
            int next=(currentCup+i)%CUP_COUNT;
            if("empty".equals(cupColors[next])){
                currentCup=next;
                spinnerTarget=currentCup*COUNTS_PER_CUP;
                integral=0;
                justScanned=false;
                return;
            }
        }
    }

    private boolean spinnerAtTarget() { return Math.abs(spinner.getCurrentPosition() - spinnerTarget) <= DEADZONE; }

    private boolean allCupsScanned() {
        for(String c: cupColors) if("empty".equals(c)) return false;
        return true;
    }

    private void sendTelemetry() {
        telemetry.addData("Spinner Pos", spinner.getCurrentPosition());
        telemetry.addData("Spinner Target", spinnerTarget);
        telemetry.addData("Current Cup", currentCup+1);
        for(int i=0;i<CUP_COUNT;i++) telemetry.addData("Cup "+(i+1), cupColors[i]);
        telemetry.update();
    }
}
