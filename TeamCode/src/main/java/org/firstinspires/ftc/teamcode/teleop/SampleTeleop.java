package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.support.PIDRobot;
import org.firstinspires.ftc.teamcode.support.Robot;

@TeleOp(name="SampleTeleop", group=" Sample")

public class SampleTeleop extends LinearOpMode {

    final double PRECISE_MOVEMENT = 0.5;
    final double TRIGGER_DEADZONE = 0.05;
    final double STICK_DEADZONE = 0;

    // Used to keep the robot facing the same direction if bumped
    boolean autoHeading = false;

    Robot robot = new Robot(this, true);

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(true);

        while(opModeInInit())
        {
            telemetry.addData(">", "Touch Play to Start");
            telemetry.addLine("Don't break anything please");
            robot.readSensors();
            telemetry.update();
        }

        while(opModeIsActive())
        {
            robot.readSensors();

            gamepad1Controls();

            gamepad2Controls();

            telemetry.addData("Linear slide current position: ", robot.linearSlide.getCurrentPosition());
            telemetry.addData("ClimbL current position: ", robot.climbL.getCurrentPosition());
            telemetry.addData("ClimbR current position: ", robot.climbR.getCurrentPosition());
            telemetry.addData("Arm current position: ", robot.arm.getCurrentPosition());
            telemetry.addLine();
            telemetry.addLine("Kai was here");

            telemetry.update();
        }
    }

    /**
     * gamepad1 Controls
     *  - Left Joystick for Drive/Strafe
     *  - Right Joystick for Turn
     *  - D-Pad for controlled movement
     *  - Buttons for snap to angle turning
     */
    private void gamepad1Controls() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        // If using the dPad, overwrite the values of x and y
        if (gamepad1.dpad_left) {
            x = -PRECISE_MOVEMENT;
        } else if (gamepad1.dpad_right) {
            x = PRECISE_MOVEMENT;
        } else if (gamepad1.dpad_up) {
            y = -PRECISE_MOVEMENT;
        } else if (gamepad1.dpad_down) {
            y = PRECISE_MOVEMENT;
        }

        // Turn off Auto Heading when driver trying to turn
        if (Math.abs(turn) > 0.05) {
            autoHeading = false;
        } else
        {
            // Driver not turning the robot and not locked into heading
            if(!autoHeading && Math.abs(robot.getTurnRate()) < 2.0)
            {
                robot.yawController.reset(robot.getHeading());
                autoHeading = true;
            }
        }

        // 90 degree rotations from starting orientation
        if(gamepad1.triangle)
        {
            robot.yawController.reset(0);
            autoHeading = true;
        }
        else if(gamepad1.square)
        {
            robot.yawController.reset(Math.PI/2);
            autoHeading = true;
        }
        else if(gamepad1.cross)
        {
            robot.yawController.reset(Math.PI);
            autoHeading = true;
        }
        else if(gamepad1.circle)
        {
            robot.yawController.reset(-Math.PI / 2);
            autoHeading = true;
        }
        if (gamepad1.right_trigger>TRIGGER_DEADZONE){
            robot.climb(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger>TRIGGER_DEADZONE) {
            robot.climb(-gamepad1.left_trigger);
        } else {
            robot.climb(0);
        }


        // Telemetry on desired heading and current heading
        telemetry.addData("Heading :: ",robot.yawController.getSetpoint());
        telemetry.addData("Current Angle :: ",robot.heading);

        if(autoHeading)
        {
            // turn = robot.yawController.getOutput(robot.getHeading());
        }
        robot.moveRobot(x,y,turn,1);
    }

    private void gamepad2Controls() {
        // Claw opening and closing
        if (gamepad2.triangle){
            robot.clawOpen(false);
        } else if (gamepad2.circle) {
            robot.clawOpen(true);
        }

        // Linear slide moving in and out
        if (gamepad2.right_trigger>TRIGGER_DEADZONE){
            robot.moveLinearSlide(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger>TRIGGER_DEADZONE) {
            robot.moveLinearSlide(-gamepad2.left_trigger);
        } else {
            robot.moveLinearSlide(0);
        }

        // Arm moving up and down
        if (Math.abs(gamepad2.left_stick_y)>STICK_DEADZONE){
            robot.moveArm(-gamepad2.left_stick_y);
        } else {
            robot.arm.setPower(0.05);
        }

        // Claw moving up and down
        if (Math.abs(gamepad2.right_stick_y)>STICK_DEADZONE){
            robot.moveWristPitch(gamepad2.right_stick_y);
        }
    }
}