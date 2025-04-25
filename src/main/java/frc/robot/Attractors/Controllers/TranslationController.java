package frc.robot.Attractors.Controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Attractors.Attractor;

public class TranslationController 
{
    public final Attractor referenceAttractor;

    private PIDController translationXPID = new PIDController(3, 0, 0);
    private PIDController translationYPID = new PIDController(3, 0, 0);

    public TranslationController(Attractor referenceAttractor)
    {
        this.referenceAttractor = referenceAttractor;

        this.translationXPID.setTolerance(0.0127);
        this.translationYPID.setTolerance(0.0127);
    }

    /**
     * Calculates the desired movement based on the provided attractor.
     * @param rPose The current robots position and rotation.
     * @return The PID-controlled "chassis-speeds" like translation of the robot represented as a Translation2D
     */
    public Translation2d calculate(Pose2d rPose)
    {
        final double magnitude = this.referenceAttractor.getMagnitude(rPose.getTranslation());
        if (magnitude == 0)
        {
            // Out of range, do not impose any movement.
            this.translationXPID.reset();
            this.translationYPID.reset();
            return new Translation2d(0, 0);
        }

        final Translation2d targetPose = this.referenceAttractor.getPosition(rPose.getTranslation());

        final Translation2d desiredSpeed = new Translation2d(
            this.translationXPID.calculate(rPose.getX(), targetPose.getX()),
            this.translationYPID.calculate(rPose.getY(), targetPose.getY())
        );
        return desiredSpeed;
    }
}