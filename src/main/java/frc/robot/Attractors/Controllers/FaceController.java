package frc.robot.Attractors.Controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Attractors.Attractor;

// Combines the capabilities of the Line and Rotation attractor into one!
public class FaceController implements Attractor
{
    private RotationController rotationController;
    private TranslationController translationController;

    public final Attractor referenceAttractor;

    public FaceController(Rotation2d desiredRotation, Attractor attractor)
    {
        this.referenceAttractor = attractor;
        this.rotationController = new RotationController(desiredRotation, attractor);
        this.translationController = new TranslationController(attractor);
    }

    public boolean isAtRotation()
    {
        return this.rotationController.isAtRotation();
    }

    public Translation2d getPosition(Pose2d rPose)
    {
        return this.translationController.referenceAttractor.getPosition(rPose.getTranslation());
    }

    public ChassisSpeeds calculate(Pose2d rPose)
    {
        Rotation2d rotationResult = this.rotationController.calculate(rPose);
        Translation2d translationResult = this.translationController.calculate(rPose);

        return new ChassisSpeeds(
            translationResult.getX(),
            translationResult.getY(),
            rotationResult.getRadians()
        );
    }

    @Override
    public double getMagnitude(Translation2d rPose) {
        return this.referenceAttractor.getMagnitude(rPose);
    }

    @Override
    public Translation2d getPosition(Translation2d rPose) {
        return this.referenceAttractor.getPosition(rPose);
    }
    
}
