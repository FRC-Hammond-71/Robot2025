package frc.robot.Geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// https://www.desmos.com/calculator/l02rfnrh3i

public class Face {
    private final LineSegment line;
    private final Rotation2d faceAngle;
    private final Rotation2d normal;

    public Face(LineSegment line) 
    {
        this.line = line;
        // Compute face direction (angle of the line)
        this.faceAngle = this.line.TranslationB.minus(this.line.TranslationA).getAngle();

        // Compute normal (rotate faceAngle by +90 degrees)
        this.normal = new Rotation2d(faceAngle.getRadians() + Math.PI / 2);
    }

    public Rotation2d getNormal() {
        return this.normal;
    }

    public Rotation2d getFaceAngle() {
        return this.faceAngle;
    }

    public Translation2d getStartPosition() {
        return this.line.TranslationA;
    }

    public Translation2d getEndPosition() {
        return this.line.TranslationB;
    }

    public double getDistance()
    {
        return this.line.TranslationB.getDistance(this.line.TranslationA);
    }

    public ChassisSpeeds convertRobotRelativeChassisSpeedsToFace(ChassisSpeeds robotRelativeSpeeds)
    {
        var rotated = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond).rotateBy(this.faceAngle.unaryMinus());

        return new ChassisSpeeds(rotated.getX(), rotated.getY(), robotRelativeSpeeds.omegaRadiansPerSecond);
    }
}
