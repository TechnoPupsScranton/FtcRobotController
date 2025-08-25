package org.firstinspires.ftc.team17104;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "start-backup-observation-auto", group = "competition")
public class BlackAuto2 extends LinearOpMode {

    static final double maxPosition = 0.35;
    static final double minPosition = 0.0;
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double RIGHT_ANGLE_TURN = 1.0; // 90
    static final double[] RIGHTTURN = {1, -1, RIGHT_ANGLE_TURN};
    static final double[] LEFTTURN = {-1, 1, RIGHT_ANGLE_TURN};
    static final double[] WAIT = {0, 0, 1};
    DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    DcMotor rightDrive = null;  //  Used to control the right back drive wheel
    DcMotor armTelescope = null;  //  Used to control the arm telescope motor
    Servo armElbow = null;
    CRServo armGecko = null;
    double armPower = 0;        // Desired forward telescope motor (-1 to +1)
    double elbowPosition = minPosition;        // Desired forward telescope motor (-1 to +1)
    double geckoPower = 0;// Desired forward telescope motor (-1 to +1)
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

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
        waitForStart();

        armElbow.setPosition(minPosition);
        moveRobot(-1, -1, 0.3);
//         Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

    /*
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double leftSpeed, double rightSpeed) {

        if (leftSpeed > 1) {
            leftSpeed = 1;
        }
        if (rightSpeed > 1) {
            rightSpeed = 1;
        }
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

    /**
     * Move robot based on time.
     *
     * @param leftSpeed  Speed of Left wheel for moving
     * @param rightSpeed Speed of Right wheel for moving
     * @param timeCycle  Number of seconds to run move
     */
    public void moveRobot(double leftSpeed, double rightSpeed, double timeCycle) {
//        final ElapsedTime runtime = new ElapsedTime();
        if (leftSpeed > 1) {
            leftSpeed = 1;
        }
        if (rightSpeed > 1) {
            rightSpeed = 1;
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.time() <= timeCycle) {
            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);
            sleep(CYCLE_MS);
            idle();
        }
    }

    public void moveArm(double armSpeed, double timeCycle) {
//        final ElapsedTime runtime = new ElapsedTime();

        if (armSpeed > 1) {
            armSpeed = 1;
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.time() <= timeCycle) {
            armTelescope.setPower(armSpeed);
            sleep(CYCLE_MS);
            idle();
        }
    }

    public void moveGecko(double spinDirection, double timeCycle) {
//        final ElapsedTime runtime = new ElapsedTime();

        if (spinDirection > 1) {
            spinDirection = 1;
        }
        if (spinDirection < -1) {
            spinDirection = -1;
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.time() <= timeCycle) {
            armGecko.setPower(spinDirection);
            sleep(CYCLE_MS);
            idle();
        }
    }

    public void moveRobot(double[] moveArray) {
//        final ElapsedTime runtime = new ElapsedTime();
//                        41.5
        if (moveArray[0] > 1) {
            moveArray[0] = 1;
        }
        if (moveArray[1] > 1) {
            moveArray[1] = 1;
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.time() <= moveArray[2]) {
            leftDrive.setPower(moveArray[0]);
            rightDrive.setPower(moveArray[1]);
            sleep(CYCLE_MS);
            idle();
        }
    }

}
// Black auto 2 meserments
//24
//rt
//12
//lt
//Speed
//72
//slow
//rt
//35
//-9
//rt
//90
//-5
//rt
//            RIGHT AUTO CODE
//
//        moveRobot(1, 1, 0.168);
//        moveRobot(RIGHTTURN);
//            moveRobot (1,1, 0.75);
//            moveRobot (LEFTTURN);
//            moveRobot (1,1,3);
//            moveRobot (RIGHTTURN);
//            moveRobot (1,1,1.5);
//            moveRobot (-1,-1,0.5);
//            moveRobot (RIGHTTURN);
//            moveRobot (1,1,3.5);
//            moveRobot (RIGHTTURN);
//            moveRobot (1,1,0.2);