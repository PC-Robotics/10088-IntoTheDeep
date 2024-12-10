package org.firstinspires.ftc.teamcode.support;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Robot extends PIDRobot
{
    public Robot(LinearOpMode opMode) {super(opMode, false);}

    public Servo wristPitch = null;
    public Servo wristYaw = null;
    public Servo claw = null;
    public DcMotor linearSlide = null;
    public DcMotor arm = null;
    public DcMotor climbLeft = null;
    public DcMotor climbRight = null;

    public void init() {
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

        climbLeft = myOpMode.hardwareMap.get(DcMotor.class, "climbL");
        climbLeft.setDirection(DcMotor.Direction.REVERSE);
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbRight = myOpMode.hardwareMap.get(DcMotor.class, "climbR");
        climbRight.setDirection(DcMotor.Direction.FORWARD);
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");

        super.init(true);
    }
}