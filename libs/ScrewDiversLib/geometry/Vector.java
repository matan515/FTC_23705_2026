import ScrewDriversLib.geometry.Pose2D;

public class Vector {
    private double x;
    private double y;
    private Double norm;  
    private Double angle; 
    private Pose2D P1;
    private Pose2D P2;
    
    
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
        this.norm = null;
        this.angle = null;
    }

    public static Vector fromPolar(double norm, double angle) {
        Vector v = new Vector(0, 0);
        v.norm = norm;
        v.angle = angle;
        v.x = norm * Math.cos(angle);
        v.y = norm * Math.sin(angle);
        return v;
    }
    
    public Vector(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public Vector(Pose2D P1, Pose2D P2){
        this.P1 = P1;
        this.P2 = P2;

        x = P2.getX() - P1.getX();
        y = P2.getY() - P2.getY();
    }
    
    public double getNorm() {
        if (norm == null) {
            return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        } else {
            return norm;
        }
    }
    
    public double getDistans() {
        if (angle == null) {
            return Math.toDegrees(Math.atan2(y, x));
        } else {
            return Math.toDegrees(angle);
        }
    }
    
    public double getRadians() {
        if (angle == null) {
            return Math.atan2(y, x);
        } else {
            return angle;
        }
    }
    
    public Pose2D getAsPose2D() {
        return new Pose2D(x, y, getRadians());
    }
    
    public Vector minus(Vector other) {
        double newX = this.x - other.x;
        double newY = this.y - other.y;
        return new Vector(newX, newY);
    }


    public Vector plus(Vector other) {
        double newX = this.x + other.x;
        double newY = this.y + other.y;
        return new Vector(newX, newY);
    }
    
    public double getX() {
        return x;
    }
    
    public double getY() {
        return y;
    }
}