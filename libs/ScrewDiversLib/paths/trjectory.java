import libs.ScrewDriversLib.geometry.Pose2D;
import libs.ScrewDriversLib.geometry.Vector;

public class trjectory {
    Pose2D P1;
    Pose2D p2;

    public trjectory(Pose2D P1, Pose2D p2){
        this.P1 = P1;
        this.P2 = P2;
    }

    public double[] getTrjectory(){
        Vector trajectoryAsVector = new Vector(P1,P2);
        // angle , distenc
        double[] trajectoryAngleAndDistens = {trajectoryAsVector.getRadians(), trajectoryAsVector.getNorm()};
        return trajectoryAngleAndDistens;
    }
}
