package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
    private CRServo servo0, servo1;
    // Define and set defaults for robot.
    private DriveChassis robotChassis = DriveChassis.TWO_WHEEL;
    private DriveMode robotDrive = DriveMode.POV; // Set default to POV
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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

    public RobotHardware(OpMode opMode) {
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
    public void servoInit(){
        servo0 = myOpMode.hardwareMap.get(CRServo.class, "servo0");
        servo0.setDirection(CRServo.Direction.FORWARD);
        //servo1 = myOpMode.hardwareMap.get(CRServo.class, "servo1");
        //servo1 = setDirection(CRServo.Direction.FORWARD);
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
        rF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.FORWARD);
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

    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

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
            rightFrontPower = axial + lateral - yaw;
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
