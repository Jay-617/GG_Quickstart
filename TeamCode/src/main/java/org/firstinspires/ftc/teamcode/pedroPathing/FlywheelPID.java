package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.sun.tools.doclint.HtmlTag.I;
import static com.sun.tools.javac.main.Option.D;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelPID extends OpMode {

    public DcMotorEx outtake;
    public DcMotorEx outtake2;

    public double HighVelocity = 3000;
    public double lowVelocity = 1500;

    double curTargetVelocity = HighVelocity;
    double F = 0;
    double P = 0;

    double[] stepsizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepindex = 1;

    @Override
    public void init() {

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        outtake2.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        if (gamepad2.yWasPressed()) {
            if (curTargetVelocity == HighVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = HighVelocity;
            }
        }

        if (gamepad2.bWasPressed()) {
            stepindex = (stepindex + 1) % stepsizes.length;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            F -= stepsizes[stepindex];
        }

        if (gamepad2.dpadRightWasPressed()) {
            F += stepsizes[stepindex];
        }

        if (gamepad2.dpadUpWasPressed()) {
            P += stepsizes[stepindex];
        }

        if (gamepad2.dpadDownWasPressed()) {
            P -= stepsizes[stepindex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        outtake2.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidfCoefficients
        );

        outtake.setVelocity(curTargetVelocity);
        outtake2.setVelocity(curTargetVelocity);

        double curVelocity = outtake.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepsizes[stepindex]);
    }
}
