package org.firstinspires.ftc.teamcode.pedroPathing;

import android.widget.RemoteViews;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class newFCTELEOP extends LinearOpMode {
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    public DcMotor intake = null;
    public DcMotor outtakeL = null;
    public DcMotor outtakeR = null;
    public DcMotor spinner = null;
    public Servo sorter= null;

    BNO055IMU imu;
    public void runOpMode(){
        // Initialize motors
        LF = hardwareMap.dcMotor.get("FL");
        LB = hardwareMap.dcMotor.get("BL");
        RF = hardwareMap.dcMotor.get("FR");
        RB = hardwareMap.dcMotor.get("BR");

        sorter = hardwareMap.servo.get("sorter");

        intake = hardwareMap.dcMotor.get("intake");
        outtakeL = hardwareMap.dcMotor.get("outtakeL");
        outtakeR = hardwareMap.dcMotor.get("outtakeR");
        spinner = hardwareMap.dcMotor.get("spinner");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);

        outtakeR.setDirection(DcMotor.Direction.FORWARD);
        outtakeL.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);
        imu = hardwareMap.get(IMU.class, "imu");


    }



}
