package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import TeamCode.src.main.java.or.firstinspires.ftc.teamcode.mechanisms.ArcadeDrive;

@Autonomous(name = "BlueAutoshoothome (Blocks to Java)", preselectTeleOp = "Main")
public class BlueAutoshoothome extends LinearOpMode {

    private DcMotor launcher;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private CRServo left_feeder;
    private CRServo right_feeder;

    String LAUNCH_BALLS;
    String IDLE;
    int LAUNCHER_TARGET_VELOCITY;
    String WAIT_FOR_LAUNCH;
    String SPIN_UP;
    int LAUNCHER_MIN_VELOCITY;
    String DRIVING_AWAY_FROM_GOAL;
    String LAUNCH;
    int TIME_BETWEEN_SHOTS;
    String launchState;
    String LAUNCHING;
    String DRIVING_OFF_LINE;
    ElapsedTime shotTimer;
    ElapsedTime feederTimer;
    String COMPLETE;
    String alliance;
    String autoStateMachine;
    String RED;
    String BLUE;
    int shotsToFire;
    ElapsedTime myElapsedTime;

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
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        right_feeder = hardwareMap.get(CRServo.class, "right_feeder");

        createAutoStateMachine();
        createLaunchStateMachine();
        createVariables();
        initMotors();
        while (opModeInInit()) {
            if (gamepad1.b) {
                alliance = RED;
            } else if (gamepad1.x) {
                alliance = BLUE;
            }
            telemetry.addLine("Press B for Red Alliance");
            telemetry.addLine("Press X for Blue Alliance");
            telemetry.addData("Selected Alliance", alliance);
            telemetry.update();
        }
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (autoStateMachine.equals(LAUNCH_BALLS)) {
                    launch(true);
                    autoStateMachine = WAIT_FOR_LAUNCH;
                } else if (autoStateMachine.equals(WAIT_FOR_LAUNCH)) {
                    if (launch(false)) {
                        shotsToFire += -1;
                        if (shotsToFire > 0) {
                            autoStateMachine = LAUNCH_BALLS;
                            myElapsedTime.reset();
                        } else {
                            ((DcMotorEx) launcher).setVelocity(0);
                            if (myElapsedTime.seconds() > 2) {
                                myElapsedTime.reset();
                                autoStateMachine = DRIVING_AWAY_FROM_GOAL;
                            }
                        }
                    }
                } else if (autoStateMachine.equals(DRIVING_AWAY_FROM_GOAL)) {
                    if (myElapsedTime.seconds() < 1.9) {
                        ((DcMotorEx) left_drive).setVelocity(0);
                        ((DcMotorEx) right_drive).setVelocity(-55);
                        telemetry.addData("boohoo time", myElapsedTime);
                    }
                    if (myElapsedTime.seconds() > 1.9) {
                        telemetry.addData("blabla", myElapsedTime);
                        ((DcMotorEx) left_drive).setVelocity(-100);
                        ((DcMotorEx) right_drive).setVelocity(-100);
                        if (myElapsedTime.seconds() > 3.3) {
                            telemetry.addData("blabla", myElapsedTime);
                            ((DcMotorEx) left_drive).setVelocity(0);
                            ((DcMotorEx) right_drive).setVelocity(0);
                            autoStateMachine = COMPLETE;
                        }
                        telemetry.update();
                    }
                } else if (false) {
                } else if (autoStateMachine.equals(DRIVING_OFF_LINE)) {
                    autoStateMachine = COMPLETE;
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
    private void initMotors() {
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.FORWARD);
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
        String ROTATING;

        LAUNCH_BALLS = "LAUNCH_BALLS";
        WAIT_FOR_LAUNCH = "WAIT_FOR_LAUNCH";
        DRIVING_AWAY_FROM_GOAL = "DRIVING_AWAY_FROM_GOAL";
        ROTATING = "ROTATING";
        DRIVING_OFF_LINE = "DRIVING_OFF_LINE";
        COMPLETE = "COMPLETE";
        autoStateMachine = LAUNCH_BALLS;
    }

    /**
     * Describe this function...
     */
    private void createLaunchStateMachine() {
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
        double TRACK_WIDTH_INCHES;
        double DRIVE_TOLERANCE;
        int holdSeconds;
        ElapsedTime driveTimer;

        LAUNCHER_TARGET_VELOCITY = 1190;
        LAUNCHER_MIN_VELOCITY = 1185;
        TIME_BETWEEN_SHOTS = 2;
        DRIVE_SPEED = 0.5;
        WHEEL_DIAMETER_MM = 96;
        ENCODER_TICKS_PER_REV = 537.7;
        TRACK_WIDTH_INCHES = 15.9;
        shotsToFire = 3;
        DRIVE_TOLERANCE = 0.5;
        holdSeconds = 1;
        shotTimer = new ElapsedTime();
        feederTimer = new ElapsedTime();
        driveTimer = new ElapsedTime();
        myElapsedTime = new ElapsedTime();
        RED = "RED";
        BLUE = "BLUE";
        alliance = RED;
    }

    /**
     * Describe this function...
     */
    private boolean launch(boolean shotRequested) {
        if (launchState.equals(IDLE)) {
            if (shotRequested == true) {
                launchState = SPIN_UP;
                shotTimer.reset();
            }
        } else if (launchState.equals(SPIN_UP)) {
            ((DcMotorEx) launcher).setVelocity(LAUNCHER_TARGET_VELOCITY);
            if (((DcMotorEx) launcher).getVelocity() > LAUNCHER_MIN_VELOCITY) {
                launchState = LAUNCH;
            }
        } else if (launchState.equals(LAUNCH)) {
            left_feeder.setPower(1);
            right_feeder.setPower(1);
            feederTimer.reset();
            launchState = LAUNCHING;
        } else if (launchState.equals(LAUNCHING)) {
            if (feederTimer.seconds() > 0.2) {
                left_feeder.setPower(0);
                right_feeder.setPower(0);
                if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    launchState = IDLE;
                    if (true) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
}
