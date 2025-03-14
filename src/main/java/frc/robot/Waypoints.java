package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Waypoints 
{
    // Faces of the reef.
    public static Pose2d R0F = new Pose2d(3, 4, Rotation2d.fromDegrees(0));
    public static Pose2d R1F = new Pose2d(3.67, 5.371581, Rotation2d.fromDegrees(-60));
    public static Pose2d R2F = new Pose2d(5.369094, 5.371581, Rotation2d.fromDegrees(-120));
    public static Pose2d R3F = new Pose2d(6.000000, 4, Rotation2d.fromDegrees(180));
    public static Pose2d R4F = new Pose2d(5.369094, 2.706350, Rotation2d.fromDegrees(120));
    public static Pose2d R5F = new Pose2d(3.67, 2.706350, Rotation2d.fromDegrees(60));

    public static Pose2d CSR = new Pose2d(new Translation2d(0.814296, 1.158110), Rotation2d.fromDegrees(-125.000000));
}
