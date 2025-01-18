package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.support.Robot;

@Autonomous(name="Auto", group="Sample")
public class Auto extends LinearOpMode {
    Robot robot = new Robot(this, false);

    @Override
    public void runOpMode() {
        robot.init(false);
        waitForStart();
        robot.strafe(-42, 0.5, 4);
        //Forwards and hang preload specimen
       // robot.drive(24, 0.5, 1);
        //robot.arm.setTargetPosition();

    }
}
