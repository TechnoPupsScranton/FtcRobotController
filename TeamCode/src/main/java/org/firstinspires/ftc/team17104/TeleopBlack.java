package org.firstinspires.ftc.team17104;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Pumpkin spice attack teleop", group = "competition")
public class TeleopBlack extends LinearOpMode {

    static final int CYCLE_MS = 50;     // period of each cycle
    static final double maxPosition = 0.35;
    static final double minPosition = 0.0;

    DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    DcMotor rightDrive = null;  //  Used to control the right back drive wheel
    DcMotor armTelescope = null;  //  Used to control the arm telescope motor
    Servo armElbow = null;
    CRServo armGecko = null;


    double leftPower = 0;        // Desired forward left motor (-1 to +1)
    double rightPower = 0;        // Desired forward right motor (-1 to +1)
    double armPower = 0;        // Desired forward telescope motor (-1 to +1)
    double elbowPosition = minPosition;        // Desired forward telescope motor (-1 to +1)
    double geckoPower = 0;// Desired forward telescope motor (-1 to +1)


    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armTelescope = hardwareMap.get(DcMotor.class, "arm_telescope");
        armElbow = hardwareMap.get(Servo.class, "arm_elbow");
        armGecko = hardwareMap.get(CRServo.class, "arm_gecko");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armTelescope.setDirection(DcMotor.Direction.REVERSE);
        armElbow.setDirection(Servo.Direction.REVERSE);
        armGecko.setDirection(CRServo.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            armPower = gamepad2.right_trigger - gamepad2.left_trigger;

            if (gamepad2.left_stick_y > 0) {
                if (elbowPosition >= maxPosition) {
                    elbowPosition = maxPosition;
                } else {
                    elbowPosition += 0.2;
                }
            } else if (gamepad2.left_stick_y < 0) {
                if (elbowPosition <= minPosition) {
                    elbowPosition = minPosition;
                } else {
                    elbowPosition -= 0.2;
                }
            }

            if (gamepad2.right_bumper) {
                geckoPower = 1;
            } else if (gamepad2.left_bumper) {
                geckoPower = -1;
            } else {
                geckoPower = 0;
            }

            if (gamepad2.a) {
                elbowPosition = 0.1;
            }

            armTelescope.setPower(armPower);
            armElbow.setPosition(elbowPosition);
            armGecko.setPower(geckoPower);


            //telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.addData(">", "armTelescopePower %5.2f", armPower);
            telemetry.addData(">", "elbowPosition %5.2f", elbowPosition);
            telemetry.addData(">", "geckoPosition %5.2f", geckoPower);

            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the motor to the new power and pause;
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
            leftPower = 1.0;
        }
        if (rightPower > 1.0) {
            rightPower = 1.0;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}