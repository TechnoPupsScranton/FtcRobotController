package org.firstinspires.ftc.team15447;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name = "Orange Left Auton")
public class LeftAutonomous extends LinearOpMode {

    static final int CYCLE_MS = 50;     // period of each cycle

    DcMotor leftDrive = null;  //  Used to control the left drive wheel
    DcMotor rightDrive = null;  //  Used to control the right drive wheel
    DcMotor armDrive = null;
    //DcMotor armDrive2 = null;
    //DcMotor Elbow_drive = null;
    CRServo elbowServo = null;
    Servo spin = null;

    private final ElapsedTime runtime = new ElapsedTime();

    double leftPower = 0;      // Desired forward power/speed (-1 to +1)
    double rightPower = 0;
    double armPower = 0;
    double ElbowPower = 0;
    double spinPosition = 0;


    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        //armDrive2 = hardwareMap.get(DcMotor.class, "arm_drive2");
        //Elbow_drive = hardwareMap.get(DcMotor.class, "Elbow_drive");
        elbowServo = hardwareMap.get(CRServo.class, "Elbow");
        spin = hardwareMap.get(Servo.class, "spin");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        // armDrive2.setDirection(DcMotor.Direction.FORWARD);
        //Elbow_drive.setDirection(DcMotor.Direction.REVERSE);
        spin.setDirection(Servo.Direction.FORWARD);
        elbowServo.setDirection(CRServo.Direction.REVERSE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Elbow_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();
        // Ramp motor speeds till stop pressed.

        // drive robot according to autonomous plans in Orange rocket notebook
        moveRobot(-1, -1, 1.5);
        moveRobot(0, 0, 0.2);
        moveRobot(0.5, -0.5, 1);
        moveArm(1, 0, 0, 1);
        moveRobot(1, 1, 0.6);


        // Add arm times when we know how fast each part is.

        // Turn off motor and signal done;
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

    /**
     * Move robot based on time.
     *
     * @param leftSpeed  Speed of Left wheel for moving
     * @param rightSpeed Speed of Right wheel for moving
     * @param timeCycle  Number of seconds to run move
     */
    public void moveRobot(double leftSpeed, double rightSpeed, double timeCycle) {

        if (leftSpeed > 1) {
            leftSpeed = 1;
        }
        if (rightSpeed > 1) {
            rightSpeed = 1;
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() <= timeCycle) {
            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);

            sleep(CYCLE_MS);
            idle();
        }
    }

    public void moveArm(double basePower, double elbowPosition, double clawPosition,
                        double timeCycle) {
        if (basePower > 1) {
            basePower = 1;
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() <= timeCycle) {
            armDrive.setPower(basePower);
            //armDrive2.setPower(basePower);
            //Elbow_drive.setPower(elbowPower);
            spin.setPosition(clawPosition);
            elbowServo.setPower(elbowPosition);

            sleep(CYCLE_MS);
            idle();
        }
        elbowServo.setPower(0);
    }


}