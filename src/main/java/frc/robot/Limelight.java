package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight
{
    protected static final Map<String, Limelight> RegisteredLimelights = new HashMap<String, Limelight>();

    public final String name;
    public final Optional<String[]> whitelistedTagIDs;

    private Limelight(String name, Optional<String[]> whitelistedTagIDs)
    {
        this.name = name;
        this.whitelistedTagIDs = whitelistedTagIDs;

        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#imu-modes
        LimelightHelpers.SetIMUMode(this.name, 1);
    }

    // TODO: Add a method which can use multiple limelights if we use mulitple!

    public Optional<PoseEstimate> getEstimationResult(Rotation2d robotGYRORot, ChassisSpeeds robotSpeeds)
    {
        // We are using MegaTag 2
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2

        // "MegaTag 2 provides excellent results at any distance given a single tag. 
        // This means it is perfectly viable to focus only on tags that are both relevant and within tolerance, and filter out all other tags
        // If a tag is not in the correct location, filter it out with the dynamic filter feature introduced alongside MegaTag2."

        LimelightHelpers.SetRobotOrientation(this.name, robotGYRORot.getDegrees(), Math.toDegrees(robotSpeeds.omegaRadiansPerSecond), 0, 0, 0, 0);

        PoseEstimate es = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);
        return Optional.ofNullable(es);
    }

    public Optional<Pose2d> getFilteredEstimatedPose(Rotation2d robotRot, ChassisSpeeds robotSpeeds)
    {
        Optional<PoseEstimate> es = this.getEstimationResult(robotRot, robotSpeeds);
        if (es.isEmpty()) return Optional.empty();

        if (es.get().tagCount < 2 || Math.abs(Math.toDegrees(robotSpeeds.omegaRadiansPerSecond)) > 720)
        // if (!es.get().isMegaTag2 || es.get().tagCount < 1 || Math.abs(Math.toDegrees(robotSpeeds.omegaRadiansPerSecond)) > 720)
        {
            return Optional.empty();
        }
        return Optional.of(es.get().pose);
    }

    public static void registerDevice(String name, Optional<String[]> whitelistedTagIDs)
    {
        if (RegisteredLimelights.containsKey(name))
        {
            // Already registered, do nothing.
            return;
        }
        RegisteredLimelights.put(name, new Limelight(name, whitelistedTagIDs));
    }

    public static Limelight useDevice(String name)
    {
        return RegisteredLimelights.get(name);
    }
}