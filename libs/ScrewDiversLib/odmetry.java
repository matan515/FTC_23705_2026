import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class odmetry {
    
    BNO055IMU imu;
    DcMotorEx motor;

    double ticks = motor.getCurrentPosition();
    double ticksPerRev = 0; //need to ceack what the the tec in the encder
    double wheelDiameter = 0.096; // in metars
    double wheelCircumference = Math.PI * wheelDiameter;

    public void init(HardwareMap hw, DcMotorEx motor){
        this.motor = motor;
        imu = hw.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADINS;
        imu.initialize(params);
    }

    public void getAngle(){
        return Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADINS).firstAngle;
    }

    public void getDistans(){
        return  distanceMeters = (motor.getPosition() / ticksPerRev) * wheelCircumference;
    }

    
}
