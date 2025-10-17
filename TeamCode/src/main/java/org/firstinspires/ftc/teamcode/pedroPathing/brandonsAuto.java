package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class brandonsAuto extends LinearOpMode {

    // Motors
    public DcMotor LF, RF, LB, RB, intake, outtakeL, outtakeR, spinner = null;

    IMU imu;

    int counter = 1;
    int counter_b = 1;

    // Track previous button state for toggles
    boolean lastB = false;
    boolean lastA = false;

    private final ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 312;    // eg: Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 1.85;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double LIFT_SPEED = 1;

    @Override
    public void runOpMode() throws InterruptedException{

        // Initialize the drive system variables.
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeL = hardwareMap.get(DcMotor.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotor.class, "outtakeR");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        //DRIVETRAIN CAN BE COMMENTED OUT IF NOT WORKING SMOOTHLY
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        RF.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        spinner.setDirection(DcMotor.Direction.FORWARD);
        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        outtakeL.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        //USE ONLY IF YOU HAVE ENCODERS
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d :%7d :%7d :%7d :%7d",
                RF.getCurrentPosition(),
                LF.getCurrentPosition(),
                RB.getCurrentPosition(),
                LB.getCurrentPosition(),
                spinner.getCurrentPosition(),
                outtakeR.getCurrentPosition(),
                outtakeL.getCurrentPosition(),
                intake.getCurrentPosition(),
                telemetry.update());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //BEGIN 30 SECOND AUTO
        Drive(1, 100);
        RotateRight(1,50);
        Spinner(1,100);
        //END AUTO

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opMode running.
     */

    public void Drive (double speed, long time) {
        RF.setPower(speed);
        LF.setPower(speed);
        RB.setPower(speed);
        LB.setPower(speed);
        this.sleep(time);
    }
    public void StrafeLeft (double speed, long time) {
        RF.setPower(speed);
        LF.setPower(-speed);
        RB.setPower(-speed);
        LB.setPower(speed);
        this.sleep(time);
    }
    public void StrafeRight(double speed, long time) {
        RF.setPower(-speed);
        LF.setPower(speed);
        RB.setPower(speed);
        LB.setPower(-speed);
        this.sleep(time);
    }
    public void RotateLeft(double speed, long time) {
        RF.setPower(speed);
        LF.setPower(-speed);
        RB.setPower(speed);
        LB.setPower(-speed);
        this.sleep(time);
    }
    public void RotateRight(double speed, long time) {
        RF.setPower(-speed);
        LF.setPower(speed);
        RB.setPower(-speed);
        LB.setPower(speed);
        this.sleep(time);
    }
    public void Intake(double speed, long time) {
        intake.setPower(speed);
        this.sleep(time);
    }
    public void Outake(double speed, long time) {
        outtakeR.setPower(speed);
        outtakeL.setPower(speed);
        this.sleep(time);
    }
    public void Spinner(double speed, long time) {
        spinner.setPower(speed);
        this.sleep(time);
    }


    public void encoderLift(double speed, double leftSide, double rightSide, double timeoutS) {
        /*int newLift1Target;
        int newLift2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLift1Target = lift1.getCurrentPosition() + (int) (leftSide * COUNTS_PER_INCH);
            newLift2Target = lift2.getCurrentPosition() + (int) (rightSide * COUNTS_PER_INCH);
            lift1.setTargetPosition(newLift1Target);
            lift2.setTargetPosition(newLift2Target);

            // Turn On RUN_TO_POSITION
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //KEEP INTAKE STEADY
            slide1.setPosition(0); // up positions
            slide2.setPosition(1);
            Srotate.setPosition(0.5);

            // reset the timeout time and start motion.
            runtime.reset();
            lift1.setPower(Math.abs(speed));
            lift2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (lift1.isBusy() && lift2.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d", newLift1Target);
                telemetry.addData("Currently at", " at %7d", newLift1Target,
                        lift1.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lift1.setPower(0);
            lift2.setPower(0);

            // Turn off RUN_TO_POSITION
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }*/
    }

//Arm encoders
    /*public void encoderarm(double speed, double fowardInches, double reverseInches, double timeoutS) {
        int newarmTarget;
        int newarm2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newarmTarget = arm.getCurrentPosition() + (int)(fowardInches * COUNTS_PER_INCH);
            newarm2Target = arm2.getCurrentPosition() + (int)(reverseInches * COUNTS_PER_INCH);
            arm.setTargetPosition(newarmTarget);
            arm2.setTargetPosition(newarm2Target);

            // Turn On RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            arm.setPower(Math.abs(speed));
            arm2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (arm.isBusy() && arm2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newarmTarget, newarm2Target);
                telemetry.addData("Currently at",  " at %7d :%7d", newarmTarget, newarm2Target,
                        arm.getCurrentPosition(), arm2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            arm.setPower(0);
            arm2.setPower(0);

            // Turn off RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    //lift encoder
    public void encoderlift(double speed, double fowardInches, double reverseInches, double timeoutS) {
        int newliftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newliftTarget = lift.getCurrentPosition() + (int)(fowardInches * COUNTS_PER_INCH);
            lift.setTargetPosition(newliftTarget);

            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (lift.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", newliftTarget);
                telemetry.addData("Currently at",  " at %7d", newliftTarget,
                        lift.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lift.setPower(0);

            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    //strafing encoders
    public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS){

        int newFRTarget;
        int newFLTarget;
        int newBRTarget;
        int newBLTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // For strafing one side's wheels 'attract' each other while the other side 'repels' each other
            //The positive value will make the robot go to the left with current arrangement
            newFRTarget = FR.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newFLTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBRTarget = BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBLTarget = BL.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            FR.setTargetPosition(newFRTarget);
            FL.setTargetPosition(newFLTarget);
            BR.setTargetPosition(newBRTarget);
            BL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FL.setPower(Math.abs(speed));
            FR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && BL.isBusy() && BR.isBusy() && FR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", newFLTarget, newFRTarget, newBLTarget, newBRTarget,
                        FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // Turn off RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    //private boolean opModeIsActive() {
    //}
}
     */