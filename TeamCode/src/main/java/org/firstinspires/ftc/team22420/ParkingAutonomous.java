package org.firstinspires.ftc.team22420;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class ParkingAutonomous extends LinearOpMode {

    static final int CYCLE_MS = 50;     // period of each cycle

    DcMotor leftDrive = null;  //  Used to control the left drive wheel
    DcMotor rightDrive = null;  //  Used to control the right drive wheel
    DcMotor leftArmBase = null;
    DcMotor rightArmBase = null;
    DcMotor armTelescope = null;
    //    DcMotor smallLeftHook = null;
//    DcMotor smallRightHook = null;
//    DcMotor bigHook = null;
    Servo grabServo1 = null;
    Servo grabServo2 = null;


    private ElapsedTime runtime = new ElapsedTime();
    double leftPower = 0;      // Desired forward power/speed (-1 to +1)
    double rightPower = 0;
    double armBasePower = 0;
    double armTelescopePower = 0;
    double grabServoPosition = 0;
    double smallHookPower = 0;
    double bigHookPower = 0;

    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftArmBase = hardwareMap.get(DcMotor.class, "left_arm_base");
        rightArmBase = hardwareMap.get(DcMotor.class, "right_arm_base");
        armTelescope = hardwareMap.get(DcMotor.class, "arm_telescope");
//         smallLeftHook = hardwareMap.get(DcMotor.class, "small_left_hook");
//        smallRightHook = hardwareMap.get(DcMotor.class, "small_right_hook");
//        bigHook = hardwareMap.get(DcMotor.class, "big_hook");
        grabServo1 = hardwareMap.get(Servo.class, "servo_1");
        grabServo2 = hardwareMap.get(Servo.class, "servo_2");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftArmBase.setDirection(DcMotor.Direction.FORWARD);
        rightArmBase.setDirection(DcMotor.Direction.REVERSE);
        armTelescope.setDirection(DcMotor.Direction.FORWARD);
//        smallLeftHook.setDirection(DcMotor.Direction.FORWARD);
//        smallRightHook.setDirection(DcMotor.Direction.FORWARD);
//        bigHook.setDirection(DcMotor.Direction.FORWARD);
        grabServo1.setDirection(Servo.Direction.REVERSE);
        grabServo2.setDirection(Servo.Direction.FORWARD);

        armTelescope.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();
        // Ramp motor speeds till stop pressed.

        // drive robot according to autonomous plans in gray rocket notebook
        moveArm(0, 0, 0.6, 0.5);
        moveRobot(1, 1, 0.55);

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
        while (runtime.time() <= timeCycle) {
            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);

        }
        sleep(CYCLE_MS);
        idle();
    }

    public void moveArm(double armBasePower, double armTelescopePower, double grabServoPosition,
                        double timeCycle) {

        if (armBasePower > 1) {
            armBasePower = 1;
        }
        if (armTelescopePower > 1) {
            armTelescopePower = 1;
        }
        if (grabServoPosition > 1) {
            grabServoPosition = 1;
        }
        runtime.reset();
        runtime.startTime();
        while (runtime.time() <= timeCycle) {

            leftArmBase.setPower(armBasePower);
            rightArmBase.setPower(armBasePower);
            armTelescope.setPower(armTelescopePower);
            grabServo1.setPosition(grabServoPosition);
            grabServo2.setPosition(grabServoPosition);

        }
        sleep(CYCLE_MS);
        idle();
    }
}