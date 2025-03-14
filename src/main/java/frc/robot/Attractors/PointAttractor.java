package frc.robot.Attractors;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PointAttractor implements Attractor
{
    protected final Translation2d position;
    protected final double minDistanceInMeters;

    public PointAttractor(Translation2d elementPosition, double minDistanceInMeters)
    {
        this.position = elementPosition;
        this.minDistanceInMeters = minDistanceInMeters;
    }

    @Override
    public double getMagnitude(Translation2d rPose)
    {
        double distance = rPose.getDistance(this.position);
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
        return this.position;
    }
}
