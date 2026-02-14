package libs.ScrewDriversLib.geometry;

public class Pose2D {
    double x;
    double y;
    double angle;

    public Pose2D(double x, double y, double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getDegrres(){
        return Math.toDegrees(angle);
    }

    public double getRadians(){
        return angle;
    }

    public Vector getAsVector(){
        return new Vector(x, y, angle);
    }
}
