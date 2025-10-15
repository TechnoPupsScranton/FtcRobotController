
package org.firstinspires.ftc.team15447;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@TeleOp(name = "Orange Robot 1")
public class OrangeRobot1Teleop extends LinearOpMode {

    static final double E_POS = 0;
    static final int CYCLE_MS = 50;     // period of each cycle

    DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    DcMotor rightDrive = null;  //  Used to control the right back drive wheel
    DcMotor armDrive = null;
    Servo spin = null; // spinner movement
    CRServo Elbow = null;

    double leftPower = 0;        // Desired forward left motor (-1 to +1)
    double rightPower = 0;        // Desired forward right motor (-1 to +1)
    double armPower = 0;
    double spinPotion = 0.5;
    double ElbowPosition = E_POS;

    @Override
    public void runOpMode() {


        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        spin = hardwareMap.get(Servo.class, "spin");
        Elbow = hardwareMap.get(CRServo.class, "Elbow");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.FORWARD);
        spin.setDirection(Servo.Direction.FORWARD);
        Elbow.setDirection(CRServo.Direction.REVERSE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {


            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            //armPower = gamepad2.left_stick_y;

            if (gamepad2.left_stick_y > 0) {
                armPower += 0.05;
                if (armPower > 1) {
                    armPower = 1;
                }
            } else if (gamepad2.left_stick_y < 0) {
                armPower -= 0.05;
                if (armPower < -1) {
                    armPower = -1;
                }
            } else {
                armPower = 0;
            }

            if (gamepad2.right_stick_y > 0) {
                ElbowPosition += 0.1;
                if (ElbowPosition > 1) {
                    ElbowPosition = 1;
                }
            } else if (gamepad2.right_stick_y < 0) {
                ElbowPosition -= 0.1;
                if (ElbowPosition < -1) {
                    ElbowPosition = -1;
                }
            } else {
                ElbowPosition = E_POS;
            }

            if (gamepad2.right_bumper) {
                spinPotion = 0.5;
            } else if (gamepad2.left_bumper) {
                spinPotion = 0;
            }

            //telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe,
            // turn);

            telemetry.addData(">", "Press Stop to end test.");
            telemetry.addData("Arm Position", "ElbowPosition : %f", ElbowPosition);
            telemetry.update();

            // Set the motor to the new power and pause;
            spin.setPosition(spinPotion);
            armDrive.setPower(armPower);
            moveRobot(leftPower, rightPower);
            Elbow.setPower(ElbowPosition);

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

