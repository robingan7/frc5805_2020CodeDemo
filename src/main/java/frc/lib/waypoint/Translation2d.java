
package frc.lib.waypoint;

public class Translation2d implements InterpolateSingle<Translation2d>{
    protected double x_, y_; 
    public static Translation2d default_= new Translation2d();

    public Translation2d(){
        x_ = 0;
        y_ = 0;
    }

    public Translation2d(final double x, final double y){
        x_ = x;
        y_ = y;
    }

    public Translation2d(final Translation2d other){
        x_ = other.x_;
        y_ = other.y_;
    }


    public Translation2d(final Translation2d start, final Translation2d end){
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    //------get value methods---------
    public double getNorm(){
        return Math.hypot(x_, y_);
    }
    public double getNorm2(){
        return Math.pow(x_, 2) + Math.pow(y_, 2);
    }
    //------moving medthods-----------
    public Translation2d translateby(Translation2d other){
        return new Translation2d(other);
    }

    public Translation2d rotateBy(Rotation2d rotate){
        return new Translation2d(x_ * rotate.cos() + y_ * rotate.sin(), x_ * rotate.sin() + y_ * rotate.cos());
    }

    public Translation2d inverse(){
        return new Translation2d(-x_, -y_);
    }

    @Override
    public Translation2d interpolate(final Translation2d other, double x) {
        if (x <= 0) {
            return new Translation2d(this);
        } else if (x >= 1) {
            return new Translation2d(other);
        }
        return extrapolate(other, x);
    }

    public Translation2d extrapolate(final Translation2d other, double x) {
        return new Translation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    } 
}
