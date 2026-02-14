import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import libs.ScrewDriversLib.geometry.Pose2D;
import libs.ScrewDriversLib.geometry.Vector;

public class odmetry {
    
    BNO055IMU imu;
    DcMotorEx motor;
    Pose2D currentPose;

    double ticks = motor.getCurrentPosition();
    double ticksPerRev = 0; //need to ceack what the the tec in the encder
    double wheelDiameter = 0.096; // in metars
    double wheelCircumference = Math.PI * wheelDiameter;

    public void init(HardwareMap hw, DcMotorEx motor, Pose2D currentPose){
        this.motor = motor;
        imu = hw.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADINS;
        imu.initialize(params);
    }

    private void getAngle(){
        return Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADINS).firstAngle;
    }

    private void getDistans(){
        return  distanceMeters = (motor.getPosition() / ticksPerRev) * wheelCircumference;
    }

    public Pose2D getNewPose(){
        Vector newPoseAsVector = new Vector(getDistans(), getAngle());
        currentPose = newPoseAsVector.getAsPose2D();
        return newPoseAsVector.getAsPose2D();
    }
    
}
