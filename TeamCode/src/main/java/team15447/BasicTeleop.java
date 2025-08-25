package team15447;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class BasicTeleop extends LinearOpMode {

    static final int CYCLE_MS = 50;     // period of each cycle

    DcMotor leftDrive = null;  //  Used to control the left back drive wheel
    DcMotor rightDrive = null;  //  Used to control the right back drive wheel

    double leftPower = 0;        // Desired forward left motor (-1 to +1)
    double rightPower = 0;        // Desired forward right motor (-1 to +1)

    @Override
    public void runOpMode() {


        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {


            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

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
            leftPower /= 1.0;
        }
        if (rightPower > 1.0) {
            rightPower /= 1.0;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}