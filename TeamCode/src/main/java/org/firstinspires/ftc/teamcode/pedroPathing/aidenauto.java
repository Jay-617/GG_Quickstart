package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AIDEN HAS SO MUCH SKILL")
public class aidenauto extends LinearOpMode {

    private DcMotor LF, RF, LB, RB;
    private DcMotor outtakeL, outtakeR, spinner;
    private Servo lifter;

    @Override
    public void runOpMode() {
        // ------------------------------
        // Hardware mapping
        // ------------------------------
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        outtakeL = hardwareMap.dcMotor.get("outtakeL");
        outtakeR = hardwareMap.dcMotor.get("outtakeR");
        spinner = hardwareMap.dcMotor.get("spinner");
        lifter = hardwareMap.servo.get("lifter");

        // ------------------------------
        // Motor directions
        // ------------------------------
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        outtakeL.setDirection(DcMotor.Direction.REVERSE);
        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // ------------------------------
        // 1. Move forward
        // ------------------------------
        double drivePower = 0.4;      // Increase = faster
        long forwardTime = 1500;      // Increase = farther
        setDrivePower(drivePower);
        sleep(forwardTime);
        stopDrive();

        // ------------------------------
        // 2. Turn left 130 degrees
        // ------------------------------
        double turnPower = 0.4;       // Increase = faster turn
        long turnTime = 500;          // Increase = larger angle
        turnLeft(turnPower);
        sleep(turnTime);
        stopDrive();

        double drivePower2 = 0.4;      // Increase = faster
        long forwardTime2 = 500;      // Increase = farther
        setDrivePower(drivePower2);
        sleep(forwardTime2);
        stopDrive();

        // ------------------------------
        // 2. Turn left 130 degrees
        // ------------------------------
        double turnPower2 = 0.4;       // Increase = faster turn
        long turnTime2 = 500;          // Increase = larger angle
        turnLeft(turnPower2);
        sleep(turnTime2);
        stopDrive();



        // ------------------------------
        // 3. Lift servo up
        // ------------------------------
        double liftUpPos = 0.81;      // 0.0–1.0, higher = lift goes higher
        lifter.setPosition(liftUpPos);
        sleep(500);                   // give servo time to reach position

        // ------------------------------
        // 4. Turn on outtake
        // ------------------------------
        double outLeftPower = 0.55;   // Left outtake speed
        double outRightPower = 0.65;  // Right outtake speed
        outtakeL.setPower(outLeftPower);
        outtakeR.setPower(outRightPower);
        sleep(500);                   // outtake runs before spinner

        // ------------------------------
        // 5. Spinner spins once
        // ------------------------------
        double spinPower = 0.4;       // spinner speed
        long spinTime = 500;          // spin duration
        spinner.setPower(spinPower);
        sleep(spinTime);
        spinner.setPower(0);
        sleep(1000);                  // pause after spinner spin

        // ------------------------------
        // 6. Stop all motors
        // ------------------------------
        outtakeL.setPower(0);
        outtakeR.setPower(0);
        spinner.setPower(0);
    }

    // ------------------------------
    // Helper Methods
    // ------------------------------
    private void setDrivePower(double p) {
        LF.setPower(p);
        LB.setPower(p);
        RF.setPower(p);
        RB.setPower(p);
    }

    private void stopDrive() {
        setDrivePower(0);
    }

    private void turnLeft(double p) {
        LF.setPower(-p);
        LB.setPower(-p);
        RF.setPower(p);
        RB.setPower(p);
    }
}
