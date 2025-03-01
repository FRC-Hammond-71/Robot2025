package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.opencv.core.Mat;
import org.w3c.dom.NameList;

import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    protected static final HashMap<String, Limelight> RegisteredLimelights = new HashMap<String, Limelight>();
    
    public final String name;
    public final Optional<String[]> whitelistedTags;

    //public Rotation2d robotRotation;
    //public ChassisSpeeds robotSpeeds;

    private Limelight(String name, Optional<String[]> whitelistedTags) {

        this.name = name;
        this.whitelistedTags = whitelistedTags;

        LimelightHelpers.SetIMUMode(this.name, 1);

    
    }

    public PoseEstimate getPoseEstimate(Rotation2d robotRotation, ChassisSpeeds robotSpeeds) {

        LimelightHelpers.SetRobotOrientation(this.name, robotRotation.getDegrees(), Math.abs(Math.toDegrees(robotSpeeds.omegaRadiansPerSecond)), 0, 0, 0, 0);

        return LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);
    }

    public Optional<Pose2d> getFilteredPoseOptional(Rotation2d robotRotation, ChassisSpeeds robotSpeeds) {

        PoseEstimate estimate = this.getPoseEstimate(robotRotation, robotSpeeds);

        if(!estimate.isMegaTag2 || estimate.tagCount < 1 || Math.abs(Math.toDegrees(robotSpeeds.omegaRadiansPerSecond)) > 720) {
            return Optional.empty();
        }

        return Optional.of(estimate.pose);
    }

    public static void registerDevice(String name, Optional<String[]> whitelistedTags) {
        if (!RegisteredLimelights.containsKey(name)) {
            RegisteredLimelights.put(name, new Limelight(name, whitelistedTags));
        }
    } 

    public static Limelight getDevice(String name) {
        return RegisteredLimelights.get(name);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Fiducial ID", LimelightHelpers.getFiducialID(this.name));

        super.periodic();
    }
}