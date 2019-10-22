/**
 * it presents a point on a unit circle
 */

 package frc.lib.waypoint;

 import static frc.lib.utility.Utli.EPSILON;;
 public class Rotation2d implements InterpolateSingle<Rotation2d>{
    protected double cos_,sin_;
    public static Rotation2d default_ =new Rotation2d();
    public Rotation2d(double cos, double sin, boolean need_foreced_rescale){
        if(need_foreced_rescale){
            double magnitude = Math.hypot(cos, sin);

            if(magnitude > EPSILON){
                cos_ = cos / magnitude;
                sin_ = sin / magnitude;
            }else{
                sin_ = 0;
                cos_ = 1;
            }
        }else{
            cos_ = cos;
            sin_ = sin;
        }
    }

    public Rotation2d(Rotation2d other){
        cos_ = other.cos_;
        sin_ = other.sin_;
    }

    public Rotation2d(final Translation2d direction, boolean forced_rescale){
        this(direction.x_, direction.y_,forced_rescale);
    }

    public static Rotation2d fromRadian(double angle_in_radians){
        return new Rotation2d(Math.cos(angle_in_radians),Math.sin(angle_in_radians),false);
    }

    public static Rotation2d fromAngle(double angle_in_degree){
        return fromRadian(Math.toRadians(angle_in_degree));
    }

    public Rotation2d(){
        this(1,0,false);
    }

    //-----get value method--------
    public double cos(){
        return cos_;
    }

    public double sin(){
        return sin_;
    }

    public double tan(){
        if(Math.abs(cos_)<EPSILON){// avoid dividing zero
            if(sin_>=0.0){
                return Double.POSITIVE_INFINITY;
            }else{
                return Double.NEGATIVE_INFINITY;
            }
        }else{
            return sin_ / cos_;
        }
    }

    public double getRadianFromCoord(){
        return Math.atan2(sin_, cos_);
    }

    public double getDegreeFromCoord(){
        return Math.toDegrees(getRadianFromCoord());
    }

    //---------moving methods--------
    public Rotation2d inverse(){
        return new Rotation2d(cos_, -sin_, false);
    }

    /**
     * inspire by this link
     *@see <a href="http://mathworld.wolfram.com/TrigonometricAdditionFormulas.html">
        Trigonometric Addition Formulas</a> 
     */
    public Rotation2d rotateFromAnother(final Rotation2d other){
        return new Rotation2d(cos_ * other.cos() - sin_ * other.sin(),
                            cos_ * other.sin()+sin_ * other.cos(), true);
    }

    public Rotation2d interpolate(final Rotation2d other, double x) {
        if (x <= 0) {
            return new Rotation2d(this);
        } else if (x >= 1) {
            return new Rotation2d(other);
        }
        double angle_diff = inverse().rotateFromAnother(other).getRadianFromCoord();
        return this.rotateFromAnother(Rotation2d.fromRadian(angle_diff * x));
    }

    public Rotation2d getCurrentRotation(){
        return this;
    }
 }