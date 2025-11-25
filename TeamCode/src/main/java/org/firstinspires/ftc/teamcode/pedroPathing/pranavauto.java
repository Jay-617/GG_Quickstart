package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "PRANAV AUTO JUST HES BUILT BETTER")
public class pranavauto extends LinearOpMode {

    private DcMotor LF, RF, LB, RB;
    private DcMotor outtakeL, outtakeR, spinner;
    private Servo lifter;

    @Override
    public void runOpMode() {
        // Hardware Map
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        outtakeL = hardwareMap.dcMotor.get("outtakeL");
        outtakeR = hardwareMap.dcMotor.get("outtakeR");
        spinner = hardwareMap.dcMotor.get("spinner");
        lifter = hardwareMap.servo.get("lifter");

        // Motor Directions
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        outtakeL.setDirection(DcMotor.Direction.REVERSE);
        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();


        // 1. Move Forward (Adjust distance and speed here)

        double drivePower = 0.4;      // Increase for faster drive
        long forwardTime = 1500;       // Increase for longer distance

        setDrivePower(drivePower);
        sleep(forwardTime);
        stopDrive();


        // 2. Turn Left 130 Degrees (Adjust angle and speed here)

        double turnPower = 0.4;       // Turning speed
        long turnTime = 750;          // Increase for more degrees, lower for less

        turnLeft(turnPower);
        sleep(turnTime);
        stopDrive();

        // 3. Lift Up (Adjust lifter position and delay here)

        double liftUpPos = 0.81;      // How high the lifter goes
        lifter.setPosition(liftUpPos);

        sleep(500);                   // Allow servo to move

        // 4. Turn THE FREAKING Outtake On LIL BRO (START TWEAKING HERE)

        double outLeftPower = 0.55;   // Left outtake speed
        double outRightPower = 0.65;  // Right outtake speed

        outtakeL.setPower(outLeftPower);
        outtakeR.setPower(outRightPower);

        sleep(500);                   //LET HIM COOK (OUTAKE COOKING UP TIME)

        // Spinner BE SPINNING LIKE KING VON Multiple Times With HOLES LIKE BODIES

        int spins = 3;                // How many times spinner activates
        double spinPower = 0.4;       // How strong spinner turns
        long spinTime = 100;          // Duration of each spin
        long gapTime = 300;          // Delay between spins

        for (int i = 0; i < spins; i++) {
            spinner.setPower(spinPower);
            sleep(spinTime);

            spinner.setPower(0);

            if (i < spins - 1) {
                sleep(gapTime);
            }
        }


        // 6. Stop Systems (HOW HOW TNT BLOWING UP STOPS ALL REDSTONE MACHINES)

        outtakeL.setPower(0);
        outtakeR.setPower(0);
        spinner.setPower(0);

        sleep(5000);
    }


    // Helper Methods (like how Aiden help the AURA KING who is PRANAV)

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
