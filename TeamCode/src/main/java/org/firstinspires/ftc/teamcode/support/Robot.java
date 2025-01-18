package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Robot extends PIDRobot {
    public Servo wristPitch = null;
    public Servo wristYaw = null;
    public Servo claw = null;
    public DcMotor linearSlide = null;
    public DcMotor arm = null;
    public DcMotor climbL = null;
    public DcMotor climbR = null;

    public Robot(LinearOpMode opMode, boolean isFieldCentric) {super(opMode, isFieldCentric);}

    public void init(boolean showTelemetry) {
        // define and init servos
        wristPitch = myOpMode.hardwareMap.get(Servo.class, "wristPitch");
        wristPitch.setDirection(Servo.Direction.FORWARD);

        wristYaw = myOpMode.hardwareMap.get(Servo.class, "wristYaw");
        wristYaw.setDirection(Servo.Direction.FORWARD);

        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);

        // define and init motors
        linearSlide = myOpMode.hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbL = myOpMode.hardwareMap.get(DcMotor.class, "climbL");
        climbL.setDirection(DcMotor.Direction.FORWARD);
        climbL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbR = myOpMode.hardwareMap.get(DcMotor.class, "climbR");
        climbR.setDirection(DcMotor.Direction.REVERSE);
        climbR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");

        super.init(true);
    }

    public void clawOpen(boolean open) {
        if (open) {
            claw.setPosition(0.4);
        } else {
            claw.setPosition(0);
        }
    }

    private double pitchPosition;
    public void moveWristPitch(float input) {
        pitchPosition+= input*0.0075;

        if(pitchPosition < 0.2) {
            pitchPosition = 0.2;
        } else if (pitchPosition > 0.8) {
            pitchPosition = 0.8;
        }

        wristPitch.setPosition(pitchPosition);
    }

    private int slideCurrentPos;
    public void moveLinearSlide(double throttle) {
        slideCurrentPos = linearSlide.getCurrentPosition();
        if ((slideCurrentPos<5700 && throttle>0)||(slideCurrentPos>100 && throttle<0)) {
            linearSlide.setPower(throttle);
        } else {
            linearSlide.setPower(0);
        }
    }

    public int armTargetPitch = 0;
    private double armCurrentPitch;
    private boolean targetPitchAcceptable=true;

    public void moveArm(float input) {
        if (targetPitchAcceptable) {
            armTargetPitch += Math.round(input * 50);
        } else if (armTargetPitch>6000) {
            armTargetPitch = 6000;
        } else {
            armTargetPitch = 0;
        }
        armCurrentPitch = arm.getCurrentPosition();
        targetPitchAcceptable = armTargetPitch>=0 && armTargetPitch<=6000;

        if ((armCurrentPitch>0 && targetPitchAcceptable)||(armCurrentPitch<6000 && targetPitchAcceptable)) {
            arm.setTargetPosition(armTargetPitch);
        }
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    private int climbLCurrentPos;
    private int climbRCurrentPos;
    public void climb(double throttle) {
        climbLCurrentPos = climbL.getCurrentPosition();
        climbRCurrentPos = climbR.getCurrentPosition();
        if (climbLCurrentPos<8000 && climbRCurrentPos< 8000 && throttle>0) {
            climbL.setPower(throttle);
            climbR.setPower(throttle);
        } else if (climbLCurrentPos>0 && climbRCurrentPos>50 && throttle<0) {
            climbL.setPower(throttle);
            climbR.setPower(throttle);
        }
        else {
            climbL.setPower(0);
            climbR.setPower(0);
        }
    }
}