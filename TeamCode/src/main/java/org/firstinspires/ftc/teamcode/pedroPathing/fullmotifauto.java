package org.firstinspires.ftc.teamcode.pedroPathing;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;




// ===== PEDRO =====
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;


// ===== HARDWARE =====
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


// ===== LIMELIGHT & APRIL TAGS =====
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import java.util.List;


@Autonomous(name = "GEAR GURUS AUTO w/ limelight ", group = "robot")
public class fullmotifauto extends LinearOpMode {

    /* VARIABLES ESTABLISHING */




    // =========================================================
    //  PEDRO PATHING
    // =========================================================
    private Follower follower;          // Pedro follower
    private boolean pathStarted = false; // prevents restarting the same path




    // =========================================================
    //  LIMELIGHT & APRIL TAGS
    // =========================================================

    private Limelight3A limelight;
    private IMU imu;

    // Pipeline switching control (copy from your example)
    private int currentPipeline = 1;               // Start with pipeline 1 (first tag / obelisk)
    private long pipelineSwitchTime = 0;
    private static final long PIPELINE_SETTLE_MS = 250;

    // Saved tag data (persistent, only set once like your example)
    private int savedTag1ID = -1;
    private double savedTag1Tx = 0;
    private double savedTag1Ty = 0;
    private double savedTag1Ta = 0;

    private int savedTag2ID = -1;
    private double savedTag2Tx = 0;
    private double savedTag2Ty = 0;
    private double savedTag2Ta = 0;

    //create new instance of string array for motif
    // cup color memory (same as teleop)
    private String[] cupColors = new String[CUP_COUNT];


    // =========================================================
    //  DRIVE BASE (used ONLY to lock the robot during shooting stage)
    // =========================================================
    private DcMotor LF, RF, LB, RB;




    // =========================================================
    //  MECHANISMS
    // =========================================================
    private DcMotor spinner;   // cup carousel motor
    private DcMotor intake2;   // intake motor (we turn this on after shoot stage #1)
    private DcMotor outtake;   // shooter motor 1
    private DcMotor outtake2;  // shooter motor 2
    private Servo uppies;      // LIFTER SERVO (feeds balls into outtake)




    // =========================================================
    //  FIELD POSES (your original Pedro poses)
    // =========================================================
    private final Pose startPose = new Pose(24.128, 122.816, Math.toRadians(90));
    private final Pose p1 = new Pose(62.912, 122.784, Math.toRadians(90));
    private final Pose p2 = new Pose(58.368, 93.056, Math.toRadians(130));
    private final Pose p3 = new Pose(17.512, 84.176);
    private final Pose p4 = new Pose(48.768, 101.568, Math.toRadians(150));




    // =========================================================
    //  PATH CHAINS
    // =========================================================
    private PathChain path1, path2, path3, path4;




    // =========================================================
    //  HIGH-LEVEL AUTO STATES
    // =========================================================
    private enum PathState {
        PATH1,      // run path 1
        PATH2,      // run path 2 (outtake turns ON while moving)
        SHOOT1,     // shooting stage #1 (base locked)
        PATH3,      // run path 3 (intake ON after shoot1)
        PATH4,      // run path 4 (intake still ON)
        SHOOT2,     // shooting stage #2 (base locked)
        STOP        // done
    }

    private PathState pathState = null;

    // =========================================================
    //  SHOOT STAGE FLAGS / RULES
    // =========================================================
    private boolean inShootStage = false;          // when true, base MUST NOT move
    private boolean outtakeLatchedOn = false;      // outtake turns on and stays on
    private boolean intakeEnabledAfterShoot1 = false; // intake turns on only after shoot stage #1


    // =========================================================
    //  LIFTER (uppies) POSITIONS
    // =========================================================
    private static final double UPPIES_UP = 0.6;    // lifter up position
    private static final double UPPIES_DOWN = 0.0;  // lifter down position




    // =========================================================
    //  SHOOT STAGE TIMING (YOUR SPEC)
    //  IMPORTANT: These sleeps are sequential, so they DO NOT overlap.
    //    - LIFTER sleep is its own delay after lifter moves.
    //    - SPINNER wait is its own delay after spinner target changes.
    // =========================================================
    private static final long LIFTER_SLEEP_MS = 500; // changed from 200 -> 500 (your request)
    private static final long SPINNER_SLEEP_MS = 400; // spinner wait (separate from lifter sleeps)




    // =========================================================
    //  SPINNER / CUP CONSTANTS
    // =========================================================
    private static final int COUNTS_PER_REV = 8192;
    private static final int CUP_COUNT = 3;
    private static final int COUNTS_PER_CUP = COUNTS_PER_REV / CUP_COUNT;

    private static final int CUP_TICK_0 = 0;
    private static final int CUP_TICK_1 = 2730;
    private static final int CUP_TICK_2 = 2730 * 2;




    // =========================================================
    //  SPINNER PID CONSTANTS (from your teleop)
    // =========================================================
    private static final double kP = 0.0026;
    private static final double kI = 0.000015;
    private static final double kD = 0.00009;
    private static final double HOLD_POWER = 0.08;
    private static final int DEADZONE = 4;
    private static final int SLOW_ZONE = 220;




    // =========================================================
    //  SPINNER SPEED LIMIT (minimal slowdown: clamp power only)
    // =========================================================
    private static final double MAX_SPINNER_POWER = 0.25; // lower than 0.25 so it moves slower




    // =========================================================
    //  SPINNER PID STATE
    // =========================================================
    private int spinnerTarget = 0;  // where we want the spinner to go (encoder ticks)
    private double integral = 0;    // integral accumulator
    private double lastError = 0;   // last error for derivative term




    // =========================================================
    //  CUP INDEX TRACKING
    //  This persists across BOTH shooting stages so we spin correctly.
    // =========================================================
    private int globalCupIndex = 0;




    @Override
    public void runOpMode() {




        // ---------------------------------------------------------
        // Create follower + set start pose
        // ---------------------------------------------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        // ---------------------------------------------------------
        // Initialize Limelight to run
        // ---------------------------------------------------------

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");  // Match your config name exactly!
        limelight.pipelineSwitch(1);  // Start on pipeline 1 (your first tag)

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(orientation));

        // ---------------------------------------------------------
        // Build Pedro paths
        // ---------------------------------------------------------
        buildPaths();




        // ---------------------------------------------------------
        // Map DRIVE motors (used to lock base during shoot stage)
        // ---------------------------------------------------------
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");




        // Drive directions (match your teleop)
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);




        // Brake so robot doesn't drift when locked
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // ---------------------------------------------------------
        // Map MECHANISMS
        // ---------------------------------------------------------
        spinner = hardwareMap.dcMotor.get("spinner");
        intake2 = hardwareMap.dcMotor.get("intake2");
        outtake = hardwareMap.dcMotor.get("outtake");
        outtake2 = hardwareMap.dcMotor.get("outtake2");
        uppies = hardwareMap.servo.get("uppies");




        // Mechanism directions (match your teleop)
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);




        // Spinner encoder setup
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // Start lifter down
        uppies.setPosition(UPPIES_DOWN);



        telemetry.addLine("AUTO READY");
        telemetry.update();

        // ---------------------------------------------------------
        // initialize cups as preloaded pattern green, purple, purple
        // ---------------------------------------------------------

        cupColors[0] = "green";
        cupColors[1] = "purple";
        cupColors[2] = "purple";

        // ---------------------------------------------------------
        // Limelight starts before start
        // ---------------------------------------------------------

        limelight.start();
        pipelineSwitchTime = System.currentTimeMillis();

        // ---------------------------------------------------------
        // Start!
        // ---------------------------------------------------------

        waitForStart();
        if (isStopRequested()) return;

        // Start state
        pathState = PathState.PATH1;

        // ---------------------------------------------------------
        // Main loop
        // ---------------------------------------------------------
        while (opModeIsActive()) {

            // If we are NOT in shooting stage, allow Pedro to update driving
            // If we ARE in shooting stage, lock base to 0 power
            if (!inShootStage) {
                follower.update();
            } else {
                setDrive(0, 0, 0, 0);
            }




            // Run our state machine
            autonomousUpdate();




            // Always run spinner PID (keeps spinner moving during waits)
            updateSpinnerPID();




            // Intake rule:
            // - OFF until shoot stage #1 completes
            // - ON during PATH3 + PATH4 only
            // - OFF during shoot stages (so it doesn't fight your shooter)
            if (intakeEnabledAfterShoot1 && !inShootStage) {
                intake2.setPower(0.8);
            } else {
                intake2.setPower(0);
            }

            /* !!! Limelight saving tag logic !!! */

            // Update IMU heading for helping with accurate botPose
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(Math.toRadians(angles.getYaw(AngleUnit.DEGREES)));  // or .getYaw() if no unit

            // Skip if pipeline just switched (give time to settle)
            if (System.currentTimeMillis() - pipelineSwitchTime < PIPELINE_SETTLE_MS) {
                telemetry.addLine("Pipeline switching...");

            } else {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    List<FiducialResult> fiducials = result.getFiducialResults();

                    for (FiducialResult fr : fiducials) {
                        int detectedID = fr.getFiducialId();

                        // ===== PATTERN SELECTION FROM TAG =====

//                        if (detectedID == 21) {
//                            cupColors[0] = "green";
//                            cupColors[1] = "purple";
//                            cupColors[2] = "purple";
//                        }
//
//                        if (detectedID == 22) {
//                            cupColors[0] = "purple";
//                            cupColors[1] = "green";
//                            cupColors[2] = "purple";
//                        }
//
//                        if (detectedID == 23) {
//                            cupColors[0] = "purple";
//                            cupColors[1] = "purple";
//                            cupColors[2] = "green";
//                        }
                        
                        // FIRST TAG logic (pipeline 1) — save only if not yet saved
                        if (currentPipeline == 1 && savedTag1ID == -1) {
                            savedTag1ID = detectedID;
                            savedTag1Tx = fr.getTargetXDegrees();
                            savedTag1Ty = fr.getTargetYDegrees();
                            savedTag1Ta = fr.getTargetArea();

                            // Switch to pipeline 0 for second tag
                            limelight.pipelineSwitch(0);
                            currentPipeline = 0;
                            pipelineSwitchTime = System.currentTimeMillis();
                            break;  // Only process one per frame during switch
                        }

                        // SECOND TAG logic (pipeline 0) — save only if not yet saved & different ID
                        else if (currentPipeline == 0 && savedTag2ID == -1 && detectedID != savedTag1ID) {
                            savedTag2ID = detectedID;
                            savedTag2Tx = fr.getTargetXDegrees();
                            savedTag2Ty = fr.getTargetYDegrees();
                            savedTag2Ta = fr.getTargetArea();
                            break;
                        }
                    }
                }
            }

            // gives us both obelisk and goal tag
            telemetry.addLine("Limelight AprilTags:");
            telemetry.addData("1st ID", savedTag1ID == -1 ? "Not Detected" : savedTag1ID);
            telemetry.addData("2nd ID", savedTag2ID == -1 ? "Not Detected" : savedTag2ID);

            /* telemetry.addLine("Second Tag values (x/y/a):");
            telemetry.addData(" Tx", "%.2f", savedTag2Tx);
            telemetry.addData(" Ty", "%.2f", savedTag2Ty);
            telemetry.addData(" Ta", "%.2f", savedTag2Ta); */

            // Optional: show if both captured
            if (savedTag1ID != -1 && savedTag2ID != -1) {
                telemetry.addLine("CAPTURED");
            }

            telemetry.addData("State", pathState);
            telemetry.addData("inShootStage", inShootStage);
            telemetry.addData("OuttakeLatchedOn", outtakeLatchedOn);
            telemetry.addData("IntakeEnabledAfterShoot1", intakeEnabledAfterShoot1);
            telemetry.addData("SpinnerPos", spinner.getCurrentPosition());
            telemetry.addData("SpinnerTarget", spinnerTarget);
            telemetry.addData("CupIndex", globalCupIndex);
            telemetry.update();
        }
    }


    // =========================================================
    //  PATH BUILDER
    // =========================================================
    private void buildPaths() {




        // Path 1: start -> p1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1))
                .setLinearHeadingInterpolation(startPose.getHeading(), p1.getHeading())
                .build();




        // Path 2: p1 -> p2
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();




        // Path 3: p2 -> p3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2, p3))
                .setTangentHeadingInterpolation()
                .build();




        // Path 4: p3 -> p4
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3, p4))
                .setLinearHeadingInterpolation(Math.toRadians(90), p4.getHeading())
                .build();
    }

    // =========================================================
    //  AUTO SPINNER MOVE HELPER
    // =========================================================
    private void moveClosestColorToTick(String color, int targetTick) {

        int bestCup = -1;
        int bestDist = Integer.MAX_VALUE;

        for (int i = 0; i < CUP_COUNT; i++) {

            if (!color.equals(cupColors[i])) continue;

            int cupPos = i * COUNTS_PER_CUP;

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
            spinnerTarget = targetTick;
            integral = 0;
            lastError = 0;

            waitMsWithPIDAndBaseLock(500);
        }
    }

    // =========================================================
    //  AUTO UPPIES KICK HELPER
    // =========================================================

    private void kickUppies() {
        uppies.setPosition(UPPIES_UP);
        waitMsWithPIDAndBaseLock(LIFTER_SLEEP_MS);

        uppies.setPosition(UPPIES_DOWN);
        waitMsWithPIDAndBaseLock(LIFTER_SLEEP_MS);
    }

    // =========================================================
    //  TAG 23 SEQUENCE
    // =========================================================

    private void runTag23CupSequence() {

        // 1) kick uppies
        kickUppies();

        // 2) move purple at 2730 -> 5460
        moveClosestColorToTick("purple", CUP_TICK_2);

        // 3) kick uppies
        kickUppies();

        // 4) move next purple -> 5460
        moveClosestColorToTick("purple", CUP_TICK_2);

        // 5) final kick
        kickUppies();
    }

    // =========================================================
    //  TOP-LEVEL AUTO STATE MACHINE
    // =========================================================

    private void autonomousUpdate() {




        switch (pathState) {




            // ---------------------------
            // PATH 1
            // ---------------------------
            case PATH1:
                if (!pathStarted) {
                    follower.followPath(path1); // start driving path1
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;         // finished path1
                    pathState = PathState.PATH2; // go to path2
                }
                break;




            // ---------------------------
            // PATH 2 (OUTTAKE ON WHILE MOVING)
            // ---------------------------
            case PATH2:
                if (!pathStarted) {




                    // Turn outtake ON while moving on Path2 (your requirement)
                    setOuttakeLatched(true);




                    follower.followPath(path2); // start driving path2
                    pathStarted = true;




                } else if (!follower.isBusy()) {




                    pathStarted = false;          // finished path2
                    pathState = PathState.SHOOT1; // begin shoot stage #1
                }
                break;




            // ---------------------------
            // SHOOT STAGE #1 (BASE LOCKED)
            // ---------------------------
            case SHOOT1:




                // Enter shoot stage mode (base must not move)
                inShootStage = true;




                // Hard lock base motors (extra safety)
                setDrive(0, 0, 0, 0);




                // Run your exact shoot stage sequence
                runShootStageExact();

                if (savedTag1ID == 23) {
                    runTag23CupSequence();
                }

//                if (savedTag1ID == 21) {
//                    runTag21CupSequence();
//                }
//
//                if (savedTag1ID == 22) {
//                    runTag22CupSequence();
//                }

                // Exit shoot stage mode

                inShootStage = false;


                // After first shoot stage completes, enable intake for later movement
                intakeEnabledAfterShoot1 = true;




                // Continue to path3
                pathState = PathState.PATH3;
                break;




            // ---------------------------
            // PATH 3 (INTAKE ON NOW)
            // ---------------------------
            case PATH3:
                if (!pathStarted) {
                    follower.followPath(path3);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.PATH4;
                }
                break;




            // ---------------------------
            // PATH 4 (INTAKE STILL ON)
            // ---------------------------
            case PATH4:
                if (!pathStarted) {
                    follower.followPath(path4);
                    pathStarted = true;
                } else if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.SHOOT2; // begin shoot stage #2
                }
                break;




            // ---------------------------
            // SHOOT STAGE #2 (BASE LOCKED)
            // ---------------------------
            case SHOOT2:




                // Enter shoot stage mode (base must not move)
                inShootStage = true;




                // Hard lock base motors (extra safety)
                setDrive(0, 0, 0, 0);




                // Run same exact shoot stage sequence again
                runShootStageExact();




                // Exit shoot stage mode
                inShootStage = false;




                // Done
                pathState = PathState.STOP;
                break;




            // ---------------------------
            // STOP
            // ---------------------------
            case STOP:
                // Make sure base is stopped
                setDrive(0, 0, 0, 0);
                break;
        }
    }




    // =========================================================
    //  YOUR EXACT SHOOT STAGE (runs twice)
    //
    //  Sequence you described:
    //   1) Lifter up, wait 500
    //   2) Lifter down, wait 500
    //   3) Spin 1 cup, wait 400
    //   4) Lifter up, wait 500
    //   5) Lifter down, wait 500
    //   6) Spin 1 cup, wait 400
    //   7) Lifter up, wait 500
    //   8) Lifter down, wait 500
    //   9) Spin 1 cup, wait 400
    //  10) Lifter up, wait 500
    //  11) Lifter down, wait 500
    //
    //  IMPORTANT: lifter waits do NOT "count as" spinner wait because
    //  they are separate calls (sequential, no overlap).
    // =========================================================
    private void runShootStageExact() {




        // Ensure outtake stays ON (latched) during shoot stage
        setOuttakeLatched(true);




        // Feed ball #1, then index carousel
        lifterUpWaitDownWait();
        spinOneCupAndWait();




        // Feed ball #2, then index carousel
        lifterUpWaitDownWait();
        spinOneCupAndWait();




        // Feed ball #3, then index carousel
        lifterUpWaitDownWait();
        spinOneCupAndWait();




        // Final lifter up/down (your last step)
        lifterUpWaitDownWait();
    }




    // =========================================================
    //  LIFTER STEP:
    //   - uppies up + wait 500ms
    //   - uppies down + wait 500ms
    // =========================================================
    private void lifterUpWaitDownWait() {




        // Move lifter up
        uppies.setPosition(UPPIES_UP);




        // Wait 500ms (base locked + PID still runs)
        waitMsWithPIDAndBaseLock(LIFTER_SLEEP_MS);




        // Move lifter down
        uppies.setPosition(UPPIES_DOWN);




        // Wait 500ms (base locked + PID still runs)
        waitMsWithPIDAndBaseLock(LIFTER_SLEEP_MS);
    }




    // =========================================================
    //  SPINNER STEP:
    //   - increment cup index
    //   - set spinner target to next cup
    //   - wait 400ms (base locked + PID still runs)
    // =========================================================
    private void spinOneCupAndWait() {




        // Advance cup index (0->1->2->0->...)
        globalCupIndex = (globalCupIndex + 1) % CUP_COUNT;




        // Set target encoder position for that cup
        spinnerTarget = getShortestTarget(
                spinner.getCurrentPosition(),
                globalCupIndex,
                0 // offset (use 0 unless you want a fixed tick offset like 2730)
        );




        // Reset PID accumulators so each move is clean
        integral = 0;
        lastError = 0;




        // Wait 400ms (ONLY spinner settle time; lifter time is separate)
        waitMsWithPIDAndBaseLock(SPINNER_SLEEP_MS);
    }




    // =========================================================
    //  SPINNER PID LOOP (always runs)
    //  - Uses your teleop PID
    //  - Slows movement by clamping max power
    // =========================================================
    private void updateSpinnerPID() {




        // Error = target - current
        int error = spinnerTarget - spinner.getCurrentPosition();




        // Simple anti-windup on integral
        if (Math.abs(error) < 200) integral += error;
        else integral = 0;




        // PID output
        double output =
                (kP * error) +
                        (kI * integral) +
                        (kD * (error - lastError));




        // Slow zone (gentler near target)
        if (Math.abs(error) < SLOW_ZONE) output *= 0.5;




        // Deadzone hold
        if (Math.abs(error) <= DEADZONE) {
            output = Math.signum(error) * HOLD_POWER;
        }




        // Clamp power to slow spinner (your “make it slower” request)
        output = Math.max(-MAX_SPINNER_POWER, Math.min(MAX_SPINNER_POWER, output));




        // Apply power to spinner
        spinner.setPower(output);




        // Optional: small intake assist while spinner is moving (matches your earlier code style)
        // If you want intake OFF always, delete these two lines.
        intake2.setPower(Math.abs(error) > 100 ? 0.3 : 0);




        // Save last error for D term
        lastError = error;
    }




    // =========================================================
    //  OUTTAKE CONTROL (LATCHED ON)
    //  - Once turned on, it stays on
    // =========================================================
    private void setOuttakeLatched(boolean on) {




        // Latch on if requested
        if (on) outtakeLatchedOn = true;




        // If not latched, do nothing (keeps it off)
        if (!outtakeLatchedOn) return;




        // Apply full power to both outtake motors
        outtake.setPower(1);
        outtake2.setPower(1);
    }




    // =========================================================
    //  WAIT FUNCTION THAT:
    //   - "Sleeps" for ms
    //   - Keeps base locked (drive power = 0)
    //   - Keeps spinner PID updating so spinner actually moves
    // =========================================================
    private void waitMsWithPIDAndBaseLock(long ms) {




        long end = System.currentTimeMillis() + ms;




        // Loop until time expires
        while (opModeIsActive() && System.currentTimeMillis() < end) {




            // Lock base (no movement)
            setDrive(0, 0, 0, 0);




            // Keep spinner PID alive during waits
            updateSpinnerPID();




            // Yield CPU
            idle();
        }
    }




    // =========================================================
    //  DRIVE POWER SETTER (for base lock)
    // =========================================================
    private void setDrive(double fl, double bl, double fr, double br) {
        LF.setPower(fl);
        LB.setPower(bl);
        RF.setPower(fr);
        RB.setPower(br);
    }




    // =========================================================
    //  SHORTEST TARGET (wrap-around) FOR SPINNER
    //  Ensures spinner takes shortest direction around the circle.
    // =========================================================
    private int getShortestTarget(int current, int cup, int offset) {




        // Base target for cup index
        int target = cup * COUNTS_PER_CUP + offset;




        // Find shortest diff around the circle
        int diff = target - current;
        if (diff > COUNTS_PER_REV / 2) diff -= COUNTS_PER_REV;
        if (diff < -COUNTS_PER_REV / 2) diff += COUNTS_PER_REV;




        // Return final target
        return current + diff;
    }
}
