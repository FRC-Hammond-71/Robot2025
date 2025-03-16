package frc.robot.Limelight;

import java.sql.Driver;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;

public class Limelight {
    protected static final Map<String, Limelight> RegisteredLimelights = new HashMap<>();

    private static final double MAX_DISTANCE_PER_CYCLE = Drivetrain.kMaxSpeed * 2 * Robot.kDefaultPeriod;
    private static final double MAX_VELOCITY_CHANGE_PER_CYCLE = SwerveModule.kMaxAcceleration  * 2 * Robot.kDefaultPeriod;
    private static final double MAX_ROTATION_CHANGE_PER_CYCLE = Drivetrain.kMaxAngularSpeed * 2 * Robot.kDefaultPeriod;
    private static final int sampleSize = 5;

    public final String name;

    // Increase timeConstant for less smoothing (faster updates)

    private final MedianFilter  xFilter = new MedianFilter(sampleSize);
    private final MedianFilter yFilter = new MedianFilter(sampleSize);
    private final MedianFilter  rotationFilter = new MedianFilter(sampleSize);

    private Pose2d lastFusedPose = null;

    private Limelight(String name) {
        this.name = name;
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#imu-modes
        LimelightHelpers.SetIMUMode(this.name, 1);
    }

    public static void registerDevice(String name) {
        RegisteredLimelights.putIfAbsent(name, new Limelight(name));
    }

    public static Limelight useDevice(String name) {
        return RegisteredLimelights.get(name);
    }

    public Optional<Pose2d> getRawEstimatedPose() {
        PoseEstimate es = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);
        return es == null ? Optional.empty() : Optional.ofNullable(es.pose);
    }

    public void resetPose(Pose2d initialPose)
    {
        this.lastFusedPose = initialPose;
        this.xFilter.reset();
        this.yFilter.reset();
        this.rotationFilter.reset();
    }

    // https://www.chiefdelphi.com/t/timestamp-parameter-when-adding-limelight-vision-to-odometry/455908/2
    // TODO: Include JSON Parsing
    public double getLatencyInSeconds()
    {
        return (LimelightHelpers.getLatency_Capture("limelight") + LimelightHelpers.getLatency_Pipeline("limelight")) / 1000;
    }

    public Optional<Pose2d> getMegaTag2EstimatedPose(Rotation2d robotGyro, ChassisSpeeds robotSpeeds) {
        LimelightHelpers.SetRobotOrientation(this.name, robotGyro.getDegrees(), 0, 0, 0, 0, 0);

        // PoseEstimate es = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) 
        //     ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name) 
        //     : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(this.name);

        PoseEstimate es = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name);

        // Reject estimates if too few tags ar seen or rotation is extreme
        if (es == null || !es.isMegaTag2 || es.tagCount < 1 || Math.abs(Math.toDegrees(robotSpeeds.omegaRadiansPerSecond)) > 360 * 2) {
            return Optional.empty();
        }
        return Optional.ofNullable(es.pose);
    }

    // Get pose with FULL filtering / smoothing (velocity, distance, angular, limits)
    public Optional<Pose2d> getStableEstimatedPose(Pose2d rPose, Rotation2d rGyro, ChassisSpeeds robotSpeeds) {
        Optional<Pose2d> estimatedPoseOpt = getMegaTag2EstimatedPose(rGyro, robotSpeeds);
        if (estimatedPoseOpt.isEmpty()) return Optional.empty();
    
        Pose2d estimatedPose = estimatedPoseOpt.get();
        // estimatedPose = new Pose2d(estimatedPose.getTranslation(), rPose.getRotation());
    
        // Outlier rejection
        if (lastFusedPose != null) {
            // Distance
            double distance = lastFusedPose.getTranslation().getDistance(estimatedPose.getTranslation());
            double velocity = distance / Robot.kDefaultPeriod;
            double velocityChange = Math.abs(velocity - robotSpeeds.vxMetersPerSecond);
    
            // Rotation
            double rotationChange = Math.abs(estimatedPose.getRotation().getRadians() - lastFusedPose.getRotation().getRadians());
    
            if (distance > MAX_DISTANCE_PER_CYCLE || velocityChange > MAX_VELOCITY_CHANGE_PER_CYCLE || rotationChange > MAX_ROTATION_CHANGE_PER_CYCLE) {
                lastFusedPose = estimatedPose;
                return Optional.empty(); // Reject the estimate
            }
        }
    
        // Apply Low-Pass Filter for X, Y, and rotation!
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/linear-filter.html#singlepoleiir
        double filteredX = xFilter.calculate(estimatedPose.getX());
        double filteredY = yFilter.calculate(estimatedPose.getY());
        double filteredRotation = rotationFilter.calculate(estimatedPose.getRotation().getRadians());
        filteredRotation = rGyro.getRadians();
        filteredRotation = Math.atan2(Math.sin(filteredRotation), Math.cos(filteredRotation));
    
        // Update the lastFusedPose with the new filtered values
        lastFusedPose = new Pose2d(filteredX, filteredY, Rotation2d.fromRadians(filteredRotation));
        return Optional.ofNullable(lastFusedPose);
        
    }
}