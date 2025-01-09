package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.support.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Sample Auto", group="Sample")
public class SampleAuto extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        robot.init(false);
        waitForStart();

        robot.drive(24, 0.5, 1);
    }
}
