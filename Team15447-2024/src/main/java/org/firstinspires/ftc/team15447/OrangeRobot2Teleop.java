
package org.firstinspires.ftc.team15447;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Orange Teleop 2")
public class OrangeRobot2Teleop extends LinearOpMode {

    static final double E_POS = 0.5;
    static final int CYCLE_MS = 50;     // period of each cycle

    DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    DcMotor rightDrive = null;  //  Used to control the right back drive wheel
    DcMotor armDrive = null;
    DcMotor armDrive2 = null;
    Servo spin = null; // spinner movement
    DcMotor Elbow_drive = null;

    double leftPower = 0;        // Desired forward left motor (-1 to +1)
    double rightPower = 0;        // Desired forward right motor (-1 to +1)
    double armPower = 0;
    double spinPotion = 0;
    double ElbowPower = 0;

    @Override
    public void runOpMode() {


        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive2 = hardwareMap.get(DcMotor.class, "arm_drive2");
        spin = hardwareMap.get(Servo.class, "spin");
        Elbow_drive = hardwareMap.get(DcMotor.class, "Elbow_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive2.setDirection(DcMotor.Direction.REVERSE);
        spin.setDirection(Servo.Direction.REVERSE);
        Elbow_drive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elbow_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {


            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            armPower = gamepad2.left_stick_y;
            ElbowPower = gamepad2.right_stick_y;

            if (gamepad2.a) {
                spinPotion = 0.5;
            } else {
                spinPotion = 0;
            }

            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the motor to the new power and pause;
            spin.setPosition(spinPotion);
            armDrive.setPower(armPower);
            armDrive2.setPower(armPower);
            moveRobot(leftPower, rightPower);
            Elbow_drive.setPower(ElbowPower);

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

