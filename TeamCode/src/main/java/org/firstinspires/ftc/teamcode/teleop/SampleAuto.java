package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.support.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Sample Auto", group="Sample")
public class SampleAuto extends LinearOpMode {
    Robot robot = new Robot(this, false);

    @Override
    public void runOpMode() {
        robot.init(false);
        waitForStart();

        //Forwards and hang preload specimen
       // robot.drive(24, 0.5, 1);
        //robot.arm.setTargetPosition();

    }
}
