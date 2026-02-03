package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "LittleTriangle (Blocks to Java)", preselectTeleOp = "Main")
public class LittleTriangle extends LinearOpMode {

    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor launcher;
    private CRServo left_feeder;
    private CRServo right_feeder;

    ElapsedTime myElapsedTime;
    String DRIVING_AWAY_FROM_GOAL;
    String launchState;
    String COMPLETE;
    String autoStateMachine;

    /**
     * Describe this function...
     */
    private void initMotors() {
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_feeder.setDirection(CRServo.Direction.REVERSE);
        left_feeder.setPower(0);
        right_feeder.setPower(0);
        ((DcMotorEx) launcher).setVelocityPIDFCoefficients(300, 0, 0, 10);
    }

    /**
     * Describe this function...
     */
    private void createAutoStateMachine() {
        String LAUNCH_BALLS;
        String WAIT_FOR_LAUNCH;
        String ROTATING;
        String DRIVING_OFF_LINE;

        LAUNCH_BALLS = "LAUNCH_BALLS";
        WAIT_FOR_LAUNCH = "WAIT_FOR_LAUNCH";
        DRIVING_AWAY_FROM_GOAL = "DRIVING_AWAY_FROM_GOAL";
        ROTATING = "ROTATING";
        DRIVING_OFF_LINE = "DRIVING_OFF_LINE";
        COMPLETE = "COMPLETE";
        autoStateMachine = DRIVING_AWAY_FROM_GOAL;
    }

    /**
     *
     * This file includes an autonomous file for the goBILDA® StarterBot for the
     * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a
     * differential/Skid-Steer system for robot mobility, one high-speed motor
     * driving two "launcher wheels," and two servos which feed that launcher.
     * This robot starts up against the goal and launches all three projectiles before
     * driving away off the starting line. This program leverages a "state machine"
     * - an Enum (or in Blocks a series of variables which capture the state
     * of the robot at any time. As it moves through the autonomous period
     * and completes different functions, it will move forward in the enum.
     * This allows us to run the autonomous period inside of our main robot "loop,"
     * continuously checking for conditions that allow us to move to the next step.
     */
    @Override
    public void runOpMode() {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        right_feeder = hardwareMap.get(CRServo.class, "right_feeder");

        createAutoStateMachine();
        createLaunchStateMachine();
        createVariables();
        initMotors();
        while (opModeInInit()) {
            telemetry.update();
            telemetry.addData("blabla", myElapsedTime);
            myElapsedTime.reset();
        }
        if (opModeInInit()) {
            myElapsedTime.reset();
        }
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (autoStateMachine.equals(DRIVING_AWAY_FROM_GOAL)) {
                    for (int count = 0; count < 125; count++) {
                        ((DcMotorEx) left_drive).setVelocity(-100);
                        ((DcMotorEx) right_drive).setVelocity(-100);
                        autoStateMachine = COMPLETE;
                        telemetry.update();
                    }
                }
                if (autoStateMachine.equals(COMPLETE)) {
                    ((DcMotorEx) left_drive).setVelocity(0);
                    ((DcMotorEx) right_drive).setVelocity(0);
                }

                telemetry.addData("Auto State", autoStateMachine);
                telemetry.addData("Launcher State", launchState);
                telemetry.addData("Motor Current Positions", "Left: " + left_drive.getCurrentPosition() + " Right: " + right_drive.getCurrentPosition());
                telemetry.addData("Motor Target Positions", "Left: " + left_drive.getTargetPosition() + " Right: " + right_drive.getTargetPosition());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void createLaunchStateMachine() {
        String IDLE;
        String SPIN_UP;
        String LAUNCH;
        String LAUNCHING;

        IDLE = "IDLE";
        SPIN_UP = "SPIN_UP";
        LAUNCH = "LAUNCH";
        LAUNCHING = "LAUNCHING";
        launchState = IDLE;
    }

    /**
     * Describe this function...
     */
    private void createVariables() {
        double DRIVE_SPEED;
        int WHEEL_DIAMETER_MM;
        double ENCODER_TICKS_PER_REV;
        double DRIVE_TOLERANCE;
        String RED;
        String BLUE;
        String alliance;

        DRIVE_SPEED = 0.5;
        WHEEL_DIAMETER_MM = 96;
        ENCODER_TICKS_PER_REV = 537.7;
        DRIVE_TOLERANCE = 0.5;
        myElapsedTime = new ElapsedTime();
        RED = "RED";
        BLUE = "BLUE";
        alliance = RED;
    }
}
