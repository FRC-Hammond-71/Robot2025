package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Attractors.LineAttractor;
import frc.robot.Geometry.Face;
import frc.robot.Geometry.LineSegment;

public class FieldConstants 
{
    @Deprecated
    public static Pose2d Base = new Pose2d(7.587, 7.288, Rotation2d.fromDegrees(180));     

    public static Pose2d BRightBlue = new Pose2d(7.248, 1.920, Rotation2d.fromDegrees(180));
    public static Pose2d BRightRed = FlippingUtil.flipFieldPose(BRightBlue);

    public static Pose2d BMiddleBlue = new Pose2d(7.248, 4, Rotation2d.fromDegrees(180));
    public static Pose2d BMiddleRed = FlippingUtil.flipFieldPose(BMiddleBlue);

    public static Pose2d BLeftBlue = new Pose2d(7.248, 6.00, Rotation2d.fromDegrees(180));
    public static Pose2d BLeftRed = FlippingUtil.flipFieldPose(BLeftBlue);
    
    public static final LineSegment AlgaeNetLine() 
    { 
        return FlipIfRequired(new LineSegment(new Translation2d(7.756, 7), new Translation2d(7.756, 4.739)), Alliance.Blue); 
    }
    public static final Rotation2d AlgaeNetRotation() { return FlipIfRequired(Rotation2d.fromDegrees(90), Alliance.Blue); };

    public static final Pose2d CoralStationLeftLineup() { return FlipIfRequired(new Pose2d(1.085, 7.000, Rotation2d.fromDegrees(126)), Alliance.Blue);  }
    public static final Pose2d CoralStationRightLineup() { return FlipIfRequired(new Pose2d(1.085, 1.062, Rotation2d.fromDegrees(-126)), Alliance.Blue);  }

    public static final Pose2d CoralProcessor()
    {
        return FlipIfRequired(new Pose2d(5.965, 0.593, Rotation2d.fromDegrees(180)), Alliance.Blue);
    }

    public static boolean IsFlipRequired(Alliance intendedAlliance)
    {
        var currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && currentAlliance.get() != intendedAlliance) {
            return true;
        }
        return false;
    }

    public static Pose2d FlipIfRequired(Pose2d pose, Alliance intendedAlliance)
    {
        var currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && currentAlliance.get() != intendedAlliance) {
            return FlippingUtil.flipFieldPose(pose);
        }
        return pose;
    }
    public static Rotation2d FlipIfRequired(Rotation2d rotation, Alliance intendedAlliance)
    {
        var currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && currentAlliance.get() != intendedAlliance) {
            return FlippingUtil.flipFieldRotation(rotation);
        }
        return rotation;
    }
    public static Translation2d FlipIfRequired(Translation2d position, Alliance intendedAlliance)
    {
        var currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && currentAlliance.get() != intendedAlliance) {
            return FlippingUtil.flipFieldPosition(position);
        }
        return position;
    }
    public static LineSegment FlipIfRequired(LineSegment line, Alliance intendedAlliance)
    {
        return new LineSegment(
            FlipIfRequired(line.TranslationA, intendedAlliance),
            FlipIfRequired(line.TranslationB, intendedAlliance));
    }
}
