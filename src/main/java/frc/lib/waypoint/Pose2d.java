package frc.lib.waypoint;

public class Pose2d implements InterpolateSingle<Pose2d>{
    /**
     * this class contain a translation2d and rotation2d objects
     * this can be known as a rigidtransform2d
     */
    public static Pose2d default_=new Pose2d();
    protected final Translation2d translation2d_;
    protected final Rotation2d rotation2d_;

    public Pose2d(){
        translation2d_ = new Translation2d();
        rotation2d_ = new Rotation2d();
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation){
        translation2d_=translation;
        rotation2d_=rotation;
    }

    public Pose2d(double x, double y, final Rotation2d rotation){
        translation2d_=new Translation2d(x,y);
        rotation2d_=rotation;
    }

    public Pose2d(final Pose2d other){
        translation2d_=new Translation2d(other.translation2d_);
        rotation2d_=new Rotation2d(other.rotation2d_);
    }

    public static Pose2d fromRotation2d(final Rotation2d rotation){
        return new Pose2d(new Translation2d(), rotation);
    } 

    public static Pose2d fromTranslation2d(final Translation2d translate){
        return new Pose2d(translate, new Rotation2d());
    }

    //-------get methods------------
    public Translation2d getTranslation() {
        return translation2d_;
    }

    public Rotation2d getRotation() {
        return rotation2d_;
    }

    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(
            translation2d_.translateby(other.translation2d_.rotateBy(rotation2d_)),
            rotation2d_.rotateFromAnother(other.rotation2d_));
    }

    @Override
	public Pose2d interpolate(Pose2d other, double proportion) {
		return new Pose2d(this.translation2d_.interpolate(other.translation2d_, proportion),
				this.rotation2d_.interpolate(other.rotation2d_, proportion));
	}
}