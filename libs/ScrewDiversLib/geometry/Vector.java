import  ScrewDriversLib.geometry.Pose2D;

public class Vector {
    double x;
    double y;

    public Vector(double x,double y){
        this.x = x;
        this.y = y;
    }


    double norm;
    double angle;

    public Vector(double norm, double angle){
        this.norm = norm;
        this.angle = angle;
    }

    public Vector(double x,double y,double angle){
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double getNorm(){
        if(norm == null){
            return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        }else{ 
            return norm;
        }
    }

    public double getDistans(){
        if(angle == null){
            return Math.toDegrees(Math.atan2(y,x));
        }else{
            return toDegrees(angle);
        } 
    }

    public double getRadins(){
        if(angle == null){
            return Math.atan2(y,x);
        }else{
            return angle;
        } 
    }

    public Pose2D getAsPose2D(){
        return new Pose2D(x,y, getRadins());
    }

    

}
