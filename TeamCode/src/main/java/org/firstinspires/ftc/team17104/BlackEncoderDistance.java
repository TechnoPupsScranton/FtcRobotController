/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team17104;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches,
 * timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run
 * profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "Robot: TestAuton", group = "test")

public class BlackEncoderDistance extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of
    // wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: REV HD Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 7;     // External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.625;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double maxPosition = 0.35;
    static final double minPosition = 0.0;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.6;
    DcMotor armTelescope = null;  //  Used to control the arm telescope motor
    Servo armElbow = null;
    CRServo armGecko = null;

    int stepCount = 0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        armTelescope = hardwareMap.get(DcMotor.class, "arm_telescope");
        armElbow = hardwareMap.get(Servo.class, "arm_elbow");
        armGecko = hardwareMap.get(CRServo.class, "arm_gecko");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTelescope.setDirection(DcMotor.Direction.REVERSE);
        armTelescope.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armElbow.setDirection(Servo.Direction.REVERSE);
        armGecko.setDirection(CRServo.Direction.REVERSE);
        // To drive forward, most robots need the motor on one side to be reversed, because the
        // axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two
        // lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction
        // or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        armElbow.setPosition(minPosition);
        encoderDrive(DRIVE_SPEED, 35, 35, 5.0);
        moveArm(1, 2);
        armElbow.setPosition(0.1);
        moveGecko(1, 2);
        armElbow.setPosition(minPosition);
        moveArm(-0.5, 2.65);
        encoderDrive(TURN_SPEED, 25, -25, 5.0);
        encoderDrive(DRIVE_SPEED, 15.5, 15.5, 5.0);
        armElbow.setPosition(maxPosition);
        moveGecko(-1, 1.5);
        encoderDrive(DRIVE_SPEED, 11.5, 11.5, 5.0);
        moveGecko(-1, 1.5);
        armElbow.setPosition(minPosition);
        encoderDrive(DRIVE_SPEED, 10, 10, 5.0);
        moveGecko(-1, 1.5);
        encoderDrive(DRIVE_SPEED, -27, -27, 5.0);
        encoderDrive(TURN_SPEED, -55, 55, 5.0);
        encoderDrive(DRIVE_SPEED, -5.2, -5.2, 5.0);
        moveArm(1, 2);
        armElbow.setPosition(0.1);
        moveGecko(1, 2);
        armElbow.setPosition(minPosition);
        moveArm(-0.5, 2.65);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget =
                    rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors
            // are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER
            // motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the
            // robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the
            // robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }

    /**
     * Move robot arm based on time.
     *
     * @param armSpeed  Speed of arm travel. +number is up, -number is down.
     * @param timeCycle Amount of time for travel.
     */
    public void moveArm(double armSpeed, double timeCycle) {
        stepCount++;
        armTelescope.setPower(armSpeed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeCycle)) {
            telemetry.addData("Arm", "Step %d: %4.1f S Elapsed", stepCount, runtime.seconds());
            telemetry.update();
        }
        armTelescope.setPower(0);
    }

    /**
     * Move Intake based on time.
     *
     * @param spinDirection Direction of spin. +number pickup, -number remove
     * @param timeCycle     Amount of time for spinning.
     */
    public void moveGecko(double spinDirection, double timeCycle) {
        stepCount++;
        armGecko.setPower(spinDirection);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeCycle)) {
            telemetry.addData("Gecko", "Step %d: %4.1f S Elapsed", stepCount, runtime.seconds());
            telemetry.update();
        }
        armGecko.setPower(0);
    }
}
