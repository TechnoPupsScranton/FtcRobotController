package org.firstinspires.ftc.team17104;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "startClose-high-bucket-2piece-auto", group = "competition")
public class BlackAuto extends LinearOpMode {

    static final double maxPosition = 0.35;
    static final double minPosition = 0.0;
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double[] FORWARD_SPEED = {1, 1};
    static final double[] REVERSE_SPEED = {-1, -1};
    static final double[] RIGHT_TURN = {1, -1};
    static final double[] LEFT_TURN = {-1, 1};
    DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    DcMotor rightDrive = null;  //  Used to control the right back drive wheel
    DcMotor armTelescope = null;  //  Used to control the arm telescope motor
    Servo armElbow = null;
    CRServo armGecko = null;
    private ElapsedTime runtime = new ElapsedTime();
    private int stepCount = 0;

    @Override
    public void runOpMode() {

        // Initialize robot variables
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armTelescope = hardwareMap.get(DcMotor.class, "arm_telescope");
        armElbow = hardwareMap.get(Servo.class, "arm_elbow");
        armGecko = hardwareMap.get(CRServo.class, "arm_gecko");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armTelescope.setDirection(DcMotor.Direction.REVERSE);
        armElbow.setDirection(Servo.Direction.REVERSE);
        armGecko.setDirection(CRServo.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart(); // - 8 from end - 2.5 from wall

        armElbow.setPosition(minPosition);
        moveRobot(FORWARD_SPEED, 0.5);
        moveArm(1, 2);  // Raise Arm
        armElbow.setPosition(0.1);
        moveGecko(1, 2);  //Drop Sample
        armElbow.setPosition(minPosition); // Set Arm Up
        moveArm(-0.5, 2);  // Lower Arm
        moveRobot(RIGHT_TURN, 1.8); // right turn modified
        armElbow.setPosition(maxPosition);
        moveGecko(-1, 2);
        moveRobot(FORWARD_SPEED, 0.5);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
        sleep(CYCLE_MS);
    }

    /**
     * Move robot based on time.
     *
     * @param leftSpeed  Speed of Left wheel for moving
     * @param rightSpeed Speed of Right wheel for moving
     * @param timeCycle  Number of seconds to run move
     */
    public void moveRobot(double leftSpeed, double rightSpeed, double timeCycle) {

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= timeCycle) {
            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);
            sleep(CYCLE_MS);
        }
    }

    /**
     * Move robot based on time.
     *
     * @param moveArray Array containing left and right speed for moving
     * @param timeCycle Number of seconds to run move
     */
    public void moveRobot(double[] moveArray, double timeCycle) {
        stepCount++;

        leftDrive.setPower(moveArray[0]);
        rightDrive.setPower(moveArray[1]);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeCycle) {
            telemetry.addData("Path", "Step %d - moving robot: %4.1f S Elapsed",
                    stepCount, runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * Move robot based on time.
     *
     * @param moveArray Array containing left and right speed for moving and amount of time (in seconds)
     */
    public void moveRobot(double[] moveArray) {
        stepCount++;

        leftDrive.setPower(moveArray[0]);
        rightDrive.setPower(moveArray[1]);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < moveArray[2]) {
            telemetry.addData("Path", "Step %d - moving robot: %4.1f S Elapsed",
                    stepCount, runtime.seconds());
            telemetry.update();
        }
    }

    public void moveArm(double armSpeed, double timeCycle) {
        stepCount++;

        armTelescope.setPower(armSpeed);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeCycle) {
            telemetry.addData("Path", "Step %d - telescoping arm: %4.1f S Elapsed",
                    stepCount, runtime.seconds());
            telemetry.update();
        }
    }

    public void moveGecko(double spinDirection, double timeCycle) {
        stepCount++;

        armGecko.setPower(spinDirection);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeCycle) {
            telemetry.addData("Path", "Step %d - spin gecko: %4.1f S Elapsed",
                    stepCount, runtime.seconds());
            telemetry.update();
        }
        armGecko.setPower(0);
    }


}
