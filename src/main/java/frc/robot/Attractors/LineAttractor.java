package frc.robot.Attractors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Geometry.LineSegment;

public class LineAttractor implements Attractor {

    protected final double minDistanceInMeters;
    
    // The start and end position of the line in question.
    public final Translation2d start;
    public final Translation2d end;        
    private final Translation2d vec;
    private final double slope;

    public LineAttractor(Translation2d start, Translation2d end, double minDistanceInMeters)
    {
        this.start = start;
        this.end = end;
        this.vec = this.end.minus(this.start);
        this.slope = this.vec.getY() / this.vec.getX();
        this.minDistanceInMeters = minDistanceInMeters;
    }
    public LineAttractor(LineSegment line, double minDistanceInMeters)
    {
        this(line.TranslationA, line.TranslationB, minDistanceInMeters);
    }

    public Rotation2d getPerpendicularRotation(boolean inverted) {
        if (this.slope == 0) {
            return new Rotation2d(Math.PI / 2 * (inverted ? -1 : 1)); 
        }
    
        double pSlope = -1 / this.slope;
        if (inverted) {
            pSlope = -pSlope;
        }
    
        return Rotation2d.fromRadians(Math.atan(pSlope));
    }

    @Override
    public double getMagnitude(Translation2d rPose) {

        final Translation2d cP = this.getPosition(rPose);

        final double distance = rPose.getDistance(cP);
        if (distance <= this.minDistanceInMeters)
        {
            // Robot is in range!
            return (minDistanceInMeters - distance) / minDistanceInMeters;
        }
        return 0;
    }

    @Override
    public Translation2d getPosition(Translation2d rPose) 
    {
        // This involves the dot product and projection (Linear Algebra!)
        // https://www.desmos.com/calculator/vx2sd6ufhb

        // Compute t (projection factor)
        // We could also use WPILIBs dot product
        // final double t = (rPose.minus(this.start).dot(dT)) / dT.getNorm();
        final double t_numerator = (rPose.getX() - this.start.getX()) * this.vec.getX() + (rPose.getY() - this.start.getY()) * this.vec.getY();
        final double t_denominator = Math.pow(this.vec.getX(), 2) + Math.pow(this.vec.getY(), 2);
        final double t = t_numerator / t_denominator;

        // if (t < 0 || t > 1)
        // {
        //     // Out of bounds, don't even compute magnitude.
        //     // TODO: Possibly create an optional return value for optimization if we have issues with timing.
        //     // return null;
        // }

        // The closest point on the line of rPose
        final Translation2d cP = new Translation2d(this.start.getX() + t * this.vec.getX(), this.start.getY() + t * this.vec.getY());

        return cP;
    }

}
