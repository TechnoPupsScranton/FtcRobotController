package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Orange Auton Test 2025")
public class OrangeAutonTest extends OpMode {
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void init() {
        robot.init(RobotHardware.DriveChassis.FOUR_WHEEL, RobotHardware.DriveMode.FIELD);
        robot.servoInit();
        robot.initAprilTag();
    }


    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void start() {
        robot.driveRobot(1, 0, 0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.driveRobot(0, 0, 0);

    }
}

