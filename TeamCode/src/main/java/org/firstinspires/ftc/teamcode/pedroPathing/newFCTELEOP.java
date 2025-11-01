package org.firstinspires.ftc.teamcode.pedroPathing;

import android.widget.RemoteViews;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
@Disabled
//@TeleOp
public class newFCTELEOP extends LinearOpMode {
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    public DcMotor intake = null;
    public DcMotor outtakeL = null;
    public DcMotor outtakeR = null;
    public DcMotor spinner = null;
    public Servo lifter=null;
    public Servo closer=null;


    int counter = 1;    // outtake toggle
    int counter_b = 1;  // intake toggle
    int counter_c = 1;  // spinner forward toggle (Y button)
    int counter_x = 1;  // spinner reverse toggle (X button)

    int counterC = 1;  // closer servo
    int counterL = 1;  // lifter servo



    // Track previous button states to detect new presses
    boolean lastB = false;
    boolean lastA = false;
    boolean lastY = false;
    boolean lastX = false;

    boolean lastC = false;
    boolean lastL = false;


    BNO055IMU imu;
    public void runOpMode() {
        // Initialize motors
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        lifter = hardwareMap.servo.get("lifter");
        closer = hardwareMap.servo.get("closer");

        intake = hardwareMap.dcMotor.get("intake");
        outtakeL = hardwareMap.dcMotor.get("outtakeL");
        outtakeR = hardwareMap.dcMotor.get("outtakeR");
        spinner = hardwareMap.dcMotor.get("spinner");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);

        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        outtakeL.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new
                RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        closer.setPosition(0.5);
        lifter.setPosition(.7);
        sleep(100);
        waitForStart();
        closer.setPosition(.5);
        lifter.setPosition(.7);
        sleep(100);


        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;


            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);


            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);


            if (gamepadRateLimit.hasExpired() && gamepad1.y) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }


            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);


            RB.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);
            LB.setPower(((adjustedLy - adjustedLx - rx) / max) * drivePower);
            RF.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            LF.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            //what the sigma - Joel


            // Toggle outtake with B button
            boolean currentB = gamepad2.b;
            if (currentB && !lastB) {
                counter++;
            }
            lastB = currentB;

            if (counter % 2 == 0) {
                outtakeR.setPower(1);
                outtakeL.setPower(1);
            } else {
                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }

            // Toggle intake with A button
            boolean currentA = gamepad2.a;
            if (currentA && !lastA) {
                counter_b++;
            }
            lastA = currentA;

            if (counter_b % 2 == 0) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // Toggle spinner forward (Y button)
            boolean currentY = gamepad2.y;
            if (currentY && !lastY) {
                counter_c++;
            }
            lastY = currentY;

            // Toggle spinner reverse (X button)
            boolean currentX = gamepad2.x;
            if (currentX && !lastX) {
                counter_x++;
            }
            lastX = currentX;

            // Spinner control: only one direction at a time to avoid clashing
            if (counter_c % 2 == 0 && counter_x % 2 != 0) {
                spinner.setDirection(DcMotorSimple.Direction.FORWARD);
                spinner.setPower(0.1);
                outtakeR.setPower(.1);
                outtakeL.setPower(.1);
                /// /////////////
            } else if (counter_x % 2 == 0 && counter_c % 2 != 0) {
                spinner.setDirection(DcMotorSimple.Direction.REVERSE);
                spinner.setPower(0.1);
                outtakeR.setPower(.1);
                outtakeL.setPower(.1);
                /// //////////////
            } else {
                spinner.setPower(0);
                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }

            // closer servo control
//            if (gamepad2.dpad_up) {
//                lifter.setPosition(.5);   // Adjust these positions to your servo range
//            } else {
//                lifter.setPosition(.4);
//
//            }
            //lifter
            boolean currentL = gamepad2.dpad_up;
            if (currentL && !lastL) {
                counterL++;
            }
            lastL = currentL;

            if (counterL % 2 == 0) {
                lifter.setPosition(.8);
            } else {
                lifter.setPosition(.7);
            }



        //closer
            boolean currentC = gamepad2.dpad_down;
            if (currentC && !lastC) {
                counterC++;
            }
            lastC = currentC;

            if (counterC % 2 == 0) {
                closer.setPosition(.6);
            } else {
                closer.setPosition(.5);
            }

            //
//            if (gamepad2.dpad_down){
//                closer.setPosition(.4);
//            } else {
//                closer.setPosition(.5);
//
//            }
        }
    }
}
