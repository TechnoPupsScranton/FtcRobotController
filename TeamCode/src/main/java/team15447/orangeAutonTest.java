package team15447;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Orange Auton Test")
public class orangeAutonTest extends LinearOpMode {

    static final int CYCLE_MS = 50;     // period of each cycle

    DcMotor leftDrive = null;  //  Used to control the left drive wheel
    DcMotor rightDrive = null;  //  Used to control the right drive wheel
    DcMotor armDrive = null;
    Servo elbowServo = null;
    Servo spin = null;

    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        elbowServo = hardwareMap.get(Servo.class, "Elbow");
        spin = hardwareMap.get(Servo.class, "spin");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        spin.setDirection(Servo.Direction.FORWARD);
        elbowServo.setDirection(Servo.Direction.REVERSE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();

        // Autonomous code
        waitForStart();
        //elbowServo.setPosition(1);
        //moveRobot( 1, 1, 0.5); // to move robot to the chambers.
        moveArm(0, 0.51, 0.5, 10); // to put robot arm in position.
        //moveArm( -0.5, 0, 0,0.5); // to get specimen on the high chamber.
        /*moveRobot( -0.5, -0.5, 1); // to get specimen clipped on the high chamber.
        moveArm( 0, 0, 1, 1); // let go of specimen.
        moveRobot( -1, -1, 1); // back robot up to the wall.
        moveRobot( 1, -1,0.5); // turn robot around.
        moveRobot( 1, 1, 0.75); // drive into observation zone.
        moveRobot( -1, 1, 0.5);
        moveRobot( 1, 1, 1.5);
        moveRobot( 1, -1,0.5);
        moveRobot( 1, 1, 0.25);
        moveRobot( 1, -1, 0.5);
        moveRobot( 1,1,1);*/


        //telemetry.addData(">", "Done");
        //telemetry.update();
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
        while (runtime.seconds() <= timeCycle) {
            leftDrive.setPower(leftSpeed);
            rightDrive.setPower(rightSpeed);


        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(CYCLE_MS);
        idle();
    }


    public void moveArm(double basePower, double elbowPosition, double clawPosition,
                        double timeCycle) {
        if (basePower > 1) {
            basePower = 1;
        }
        spin.setPosition(clawPosition);
        elbowServo.setPosition(elbowPosition);

        telemetry.addData("elbow position: %f", elbowServo.getPosition());
        telemetry.update();

        runtime.reset();
        runtime.startTime();
        while (runtime.seconds() <= timeCycle) {
            armDrive.setPower(basePower);
        }

        armDrive.setPower(0);
        sleep(CYCLE_MS);
        idle();
    }
}
