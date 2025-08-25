package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file defines a Java Class that performs all the setup and configuration for a robot's
 * hardware (motors and sensors).
 *
 * This one file/class can be used by ALL OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code
 * just makes calls into the class, rather than accessing the internal hardware directly. This is
 * why the objects are declared "private".
 */
public class RobotHardware {

    // Declare OpMode members.
    private OpMode myOpMode; // Get access to methods in the OpMode.
    // Define Motor objects.
    private DcMotorEx lF, rF, lB, rB;
    private IMU imu;
    // Define and set defaults for robot.
    private DriveChassis robotChassis = DriveChassis.TWO_WHEEL;
    private DriveMode robotDrive = DriveMode.POV; // Set default to POV

    // Define Chassis constants. Made public to be used with init calls.
    public enum DriveChassis {
        FOUR_WHEEL("4-Wheel"), TWO_WHEEL("2-Wheel");

        DriveChassis(String s) {
        }
    }

    // Define Driving Mode constants. Made public so they can be used with init calls.
    public enum DriveMode {
        POV("POV"), FIELD("Field");

        DriveMode(String s) {
        }
    }

    /**
     * Class constructor.
     *
     * @param opMode Reference to the OpMode
     */
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    /**
     * Initialize the robot's drive hardware.
     *
     * @param chassis DriveChassis setup.
     */
    public void init(DriveChassis chassis) {
        switch (chassis) {
            case FOUR_WHEEL:
                robotChassis = chassis;
                setFourWheelInit();
                break;
            case TWO_WHEEL:
                robotChassis = chassis;
                setTwoWheelInit();
                break;
        }
    }

    /**
     * Initialize the robot's drive hardware and drive mode.
     *
     * @param chassis DriveChassis setup.
     * @param mode    DriveMode selection
     */
    public void init(DriveChassis chassis, DriveMode mode) {
        switch (chassis) {
            case FOUR_WHEEL:
                robotChassis = chassis;
                robotDrive = mode;
                setFourWheelInit();
                break;
            case TWO_WHEEL:
                robotChassis = chassis;
                robotDrive = mode;
                setTwoWheelInit();
                break;
        }
    }

    /**
     * Initialize all the robot's two-wheel drive hardware.
     */
    private void setTwoWheelInit() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        lB = myOpMode.hardwareMap.get(DcMotorEx.class, "left_drive");
        rB = myOpMode.hardwareMap.get(DcMotorEx.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the
        // axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines
        // based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction
        // or 90 Deg drives may require direction flips
        lB.setDirection(DcMotor.Direction.REVERSE);
        rB.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to Brake when 0 power is set
        lB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData("Status", "%s Chassis", robotChassis.toString());
        myOpMode.telemetry.addData("Status", "%s Drive Mode", robotDrive.toString());
        myOpMode.telemetry.addData("Status", "Initialized!");
        myOpMode.telemetry.update();
    }

    /**
     * Initialize all the robot's four-wheel drive hardware.
     */
    private void setFourWheelInit() {

        lF = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        lB = myOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rF = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rB = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        lF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.REVERSE);
        rF.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.FORWARD);

        // We need the IMU setup if we're driving based on Field-Centric
        if (robotDrive == DriveMode.FIELD) {
            imu = myOpMode.hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match robot
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            IMU.Parameters parameters =
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);
        }

        myOpMode.telemetry.addData("Status", "Chassis! %s", robotChassis.toString());
        myOpMode.telemetry.addData("Status", "Drive Mode! %s", robotDrive.toString());
        myOpMode.telemetry.addData("Status", "Initialized!");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param drive Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param turn  Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double drive, double turn) {
        // Combine drive and turn for blended motion.
        double left = drive + turn;
        double right = drive - turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }

    /**
     * Calculates the left/right back/front motor powers required to achieve the requested
     * robot motions: Drive (Axial/Lateral motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial   Forward/Backward movement. Depends on driveMode.
     * @param lateral Left/Right movement. Depends on driveMode.
     * @param yaw     Turning movement.
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        double leftFrontPower = 0.0;
        double rightFrontPower = 0.0;
        double leftBackPower = 0.0;
        double rightBackPower = 0.0;

        if (robotDrive == DriveMode.POV) {
            double max;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            lF.setPower(leftFrontPower);
            rF.setPower(rightFrontPower);
            lB.setPower(leftBackPower);
            rB.setPower(rightBackPower);
        } else if (robotDrive == DriveMode.FIELD) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
            leftFrontPower = (rotY + rotX + yaw) / denominator;
            leftBackPower = (rotY - rotX + yaw) / denominator;
            rightFrontPower = (rotY - rotX - yaw) / denominator;
            rightBackPower = (rotY + rotX - yaw) / denominator;
        }

        setDrivePower(leftBackPower, rightBackPower, leftFrontPower, rightFrontPower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param wheels Fwd/Rev driving power (-1.0 to 1.0) +ve is forward.  Array order
     *               of wheels should be left to right, back to front
     */
    private void setDrivePower(double... wheels) {
        // Output the values to the motor drives.
        lB.setPower(wheels[0]);
        rB.setPower(wheels[1]);
        if (wheels.length >= 4) {
            lF.setPower(wheels[2]);
            rF.setPower(wheels[3]);
        }
    }
}
