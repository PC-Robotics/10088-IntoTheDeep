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
        robot.moveMotors(1, -1, -1, 1);
        sleep(2000);
        robot.moveMotors(0,0,0,0);

    }
}
