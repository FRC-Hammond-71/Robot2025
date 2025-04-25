package frc.robot.Attractors.Controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Attractors.Attractor;

public class RotationController
{
    private Rotation2d desiredRotation;
    /**
     * Attractor should be fully rotated by this magnitude (from 0-1) instead of at 1 (closest distance).
     */
    private double rotationBuffer = 1;
    private PIDController rotationPID = new PIDController(3.2, 0, 0);

    public final Attractor referenceAttractor;


    public RotationController(Rotation2d desiredRotation, Attractor attractor)
    {
        this.rotationPID.setTolerance(2);
		this.rotationPID.enableContinuousInput(-180, 180);

        // this.rotationPID.enableContinuousInput(0, rotationBuffer);
        this.desiredRotation = desiredRotation;
        this.referenceAttractor = attractor;
    }

    public boolean isAtRotation()
    {
        return this.rotationPID.atSetpoint();
    }

    /**
     * Calculates the desired rotation based on the provided attractor.
     * @param rPose The current robots position and rotation.
     * @return The rotation per second the robot should be moving.
     */
    public Rotation2d calculate(Pose2d rPose)
    {
        // https://www.desmos.com/calculator/vpoq8hbin1

        double m = this.referenceAttractor.getMagnitude(rPose.getTranslation());
        double m_f = Math.min(m, this.rotationBuffer) / this.rotationBuffer;

        // Rotation2d a = Rotation2d.fromDegrees((1 - m_f) * (-this.desiredRotation.getDegrees()) + m_f * this.desiredRotation.getDegrees());
        // // Rotation2d a = rPose.getRotation().plus(this.desiredRotation.minus(rPose.getRotation())).times(m_f);
        
        // double c_e = Math.abs(this.desiredRotation.minus(rPose.getRotation()).getRadians());
        // double a_e = Math.abs(this.desiredRotation.minus(a).getRadians());

        // if (a_e >= c_e)
        // {
        //     return rPose.getRotation();
        // }
        // else return a;

        if (m_f == 0)
        {
            this.rotationPID.reset();
            return Rotation2d.fromDegrees(0);
        }

        return Rotation2d.fromDegrees(rotationPID.calculate(rPose.getRotation().getDegrees(), this.desiredRotation.getDegrees()));

        // return (this.desiredRotation.minus(rPose.getRotation()))
        //     .times(Math.min(this.referenceAttractor.getMagnitude(rPose.getTranslation()), this.rotationBuffer) / this.rotationBuffer)
        //     .plus(rPose.getRotation());
    }
}
