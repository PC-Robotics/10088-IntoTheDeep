package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Robot extends PIDRobot {
    private Servo wristPitch = null;
    private Servo wristYaw = null;
    private Servo claw = null;
    private DcMotor linearSlide = null;
    private DcMotor arm = null;
    private DcMotor climbL = null;
    private DcMotor climbR = null;

    public Robot(LinearOpMode opMode) {
        super(opMode, false);
    }

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
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbL = myOpMode.hardwareMap.get(DcMotor.class, "climbL");
        climbL.setDirection(DcMotor.Direction.REVERSE);
        climbL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbR = myOpMode.hardwareMap.get(DcMotor.class, "climbR");
        climbR.setDirection(DcMotor.Direction.FORWARD);
        climbR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");

        super.init(true);
    }

    public void clawOpen(boolean open) {
        if (open) {
            claw.setPosition(0.5);
        } else {
            claw.setPosition(0);
        }
    }

    public void lowerWrist(boolean down) {
        if (down) {
            wristPitch.setPosition(0.125);
        } else {
            wristPitch.setPosition(0.625);
        }
    }

    public void moveLinearSlide(boolean isForwards) {

    }
}