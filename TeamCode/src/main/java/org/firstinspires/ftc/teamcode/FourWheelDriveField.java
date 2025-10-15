package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "4-Wheel Drive", group = "Field Centric")
public class FourWheelDriveField extends OpMode {

    IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx lF, rF, lB, rB;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {

        lF = hardwareMap.get(DcMotorEx.class, "leftFront");
        lB = hardwareMap.get(DcMotorEx.class, "leftBack");
        rF = hardwareMap.get(DcMotorEx.class, "rightFront");
        rB = hardwareMap.get(DcMotorEx.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");

        lF.setDirection(DcMotor.Direction.FORWARD);
        lB.setDirection(DcMotor.Direction.REVERSE);
        rF.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.REVERSE);

        // Adjust the orientation parameters to match robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters =
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * User-defined start method
     * <p>
     * This method will be called once, when the START button is pressed.
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double rightFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY + rotX - rx) / denominator;
        double rightBackPower = (rotY - rotX + rx) / denominator;

        lF.setPower(leftFrontPower);
        lB.setPower(leftBackPower);
        rF.setPower(rightFrontPower);
        rB.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }
}