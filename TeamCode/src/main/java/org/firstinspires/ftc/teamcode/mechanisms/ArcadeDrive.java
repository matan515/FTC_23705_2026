package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.media.Ringtone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import libs.ScrewDriversLib.odmetry;
import libs.ScrewDriversLib.geometry.Pose2D;

public class ArcadeDrive {
    private DcMotor leftDrive, rightDrive;
    private final double pi = Math.PI;

    odmetry odmetry;

    public void init(HardwareMap hwMap)
    {
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        odmetry = new odmetry(0);

    }


    public static Pose2D getCurrentPosition(odmetry odmetry){
        odmetry = new odmetry(0);
        return odmetry.getNewPose();
    }

    public void drive(double forward, double rotate)
    {
        double leftPower, rightPower;
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void driveDistance(double dist)
    {
        double rots = dist /(96 * pi);
        double motordist = rots * 537.7;

        leftDrive.setTargetPosition((-(int)motordist));
        rightDrive.setTargetPosition(((int)motordist));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);

    }
    // Inside ArcadeDrive.java

}
