package org.firstinspires.ftc.team22420;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop_Code extends LinearOpMode {

    static final int CYCLE_MS = 50;     // period of each cycle

    DcMotor leftDrive = null;  //  Used to control the left drive wheel
    DcMotor rightDrive = null;  //  Used to control the right drive wheel
    DcMotor leftArmBase = null;
    DcMotor rightArmBase = null;
    DcMotor armTelescope = null;
    Servo grabServo1 = null;
    Servo grabServo2 = null;

    double leftPower = 0;      // Desired forward power/speed (-1 to +1)
    double rightPower = 0;
    double armBasePower = 0;
    double armTelescopePower = 0;
    double grabServoPosition = 0.55; // Desired forward right motor (-1 to +1)

    @Override
    public void runOpMode() {


        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftArmBase = hardwareMap.get(DcMotor.class, "left_arm_base");
        rightArmBase = hardwareMap.get(DcMotor.class, "right_arm_base");
        armTelescope = hardwareMap.get(DcMotor.class, "arm_telescope");
        grabServo1 = hardwareMap.get(Servo.class, "servo_1");
        grabServo2 = hardwareMap.get(Servo.class, "servo_2");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftArmBase.setDirection(DcMotor.Direction.FORWARD);
        rightArmBase.setDirection(DcMotor.Direction.REVERSE);
        armTelescope.setDirection(DcMotor.Direction.FORWARD);
        grabServo1.setDirection(Servo.Direction.FORWARD);
        grabServo2.setDirection(Servo.Direction.REVERSE);
        armTelescope.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {


            leftPower = 0.75 * gamepad1.left_stick_y;
            rightPower = 0.75 * gamepad1.right_stick_y;
            armBasePower = -0.5 * gamepad2.left_stick_y;
            armTelescopePower = -gamepad2.right_stick_y;

            if (gamepad2.right_bumper) {
                grabServoPosition = 0;
            } else if (gamepad2.left_bumper) {
                grabServoPosition = 0.55;
            }

            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the motor to the new power and pause;
            leftArmBase.setPower(armBasePower);
            rightArmBase.setPower(armBasePower);
            armTelescope.setPower(armTelescopePower);
            grabServo1.setPosition(grabServoPosition);
            grabServo2.setPosition(grabServoPosition);
            moveRobot(leftPower, rightPower);
            sleep(CYCLE_MS);
            idle();

        }

        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }

    public void moveRobot(double leftPower, double rightPower) {
        if (leftPower > 1.0) {
            leftPower /= 1.0;
        }
        if (rightPower > 1.0) {
            rightPower /= 1.0;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

    }
}