package frc.robot.Subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight.Limelight;

public class Odometry
{
    private LocalOdometryThread localOdometryThread;
    
    private final SwerveDrivePoseEstimator poseEstimator;

    private final ArrayList<Limelight> visionContributors = new ArrayList<>();

    private Drivetrain associatedDrivetrain;

    //#region NT Publishers
    private StructPublisher<Pose2d> llRawPosePublisher = NetworkTableInstance.getDefault().getStructTopic("ll_RawPose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> llMegaTagPosePublisher = NetworkTableInstance.getDefault().getStructTopic("ll_MegaTagPose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> llStablePosePublisher = NetworkTableInstance.getDefault().getStructTopic("ll_StablePose", Pose2d.struct).publish();
    //#endregion

    public Odometry(Drivetrain associatedDrivetrain)
    {
        this.associatedDrivetrain = associatedDrivetrain;
        this.poseEstimator = new SwerveDrivePoseEstimator(associatedDrivetrain.m_kinematics,
            associatedDrivetrain.getGyroHeading(),
			associatedDrivetrain.getModulePositions(), 
            new Pose2d(0, 0, new Rotation2d(0)),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(0)),
            VecBuilder.fill(2, 2, Math.toRadians(30)));

        this.localOdometryThread = new LocalOdometryThread();

        // Ignore, only used to increase performance of odometry.
        this.odometryModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < odometryModulePositions.length; i++)
        {
            this.odometryModulePositions[i] = new SwerveModulePosition();
        }

        this.visionContributors.add(Limelight.useDevice("limelight"));
        
        this.localOdometryThread.start();
    }

    public Pose2d getEstimatedPose()
    {
        return this.poseEstimator.getEstimatedPosition();
    }

    // Ignore, only used to increase performance of odometry.
    private double[] odometryRecordOutput = new double[OdometryBuffer.kOutputSize];
    private SwerveModulePosition[] odometryModulePositions;

    // @Override
    // public void periodic() 
    // {
    //     updateLocalOdometry();

    //     updateVision();
    // }

    public void update()
    {
        SmartDashboard.putNumber("Odometry/bufferLength", this.localOdometryThread.buffer.length());
        updateLocalOdometry();
        updateVision();
    }

    private void updateLocalOdometry()
    {
        while (this.localOdometryThread.buffer.poll(odometryRecordOutput)) 
        {
            // Front Left module
            odometryModulePositions[0].distanceMeters = odometryRecordOutput[OdometryBuffer.kOutputFrontLeftDrivePosition];
            odometryModulePositions[0].angle = Rotation2d.fromDegrees(odometryRecordOutput[OdometryBuffer.kOutputFrontLeftAzimuthRotation]);
        
            // Front Right module
            odometryModulePositions[1].distanceMeters = odometryRecordOutput[OdometryBuffer.kOutputFrontRightDrivePosition];
            odometryModulePositions[1].angle = Rotation2d.fromDegrees(odometryRecordOutput[OdometryBuffer.kOutputFrontRightAzimuthRotation]);
        
            // Back Left module
            odometryModulePositions[2].distanceMeters = odometryRecordOutput[OdometryBuffer.kOutputBackLeftDrivePosition];
            odometryModulePositions[2].angle = Rotation2d.fromDegrees(odometryRecordOutput[OdometryBuffer.kOutputBackLeftAzimuthRotation]);
        
            // Back Right module
            odometryModulePositions[3].distanceMeters = odometryRecordOutput[OdometryBuffer.kOutputBackRightDrivePosition];
            odometryModulePositions[3].angle = Rotation2d.fromDegrees(odometryRecordOutput[OdometryBuffer.kOutputBackRightAzimuthRotation]);
        
            // Update pose estimator with timestamp and gyro heading
            this.poseEstimator.updateWithTime(
                odometryRecordOutput[OdometryBuffer.kOutputTimestamp], 
                Rotation2d.fromDegrees(odometryRecordOutput[OdometryBuffer.kOutputGyroHeading]), 
                odometryModulePositions);
        }
    }

    private void updateVision()
    {
        double initialTime = Timer.getFPGATimestamp();

        // TODO: Implement vision contributor weighting.
        for (Limelight ll : this.visionContributors)
        {
            Optional<Pose2d> poseEstimate;
            // if (this.associatedDrivetrain.gyroNotConnectedAlert.get())
            // {
            //     // Use MegaTag 1 due to MM2 reliance on Gyro.
            //     poseEstimate = ll.getMT1Pose();
            // }
            // else
            // {
            //     // MegaTag 2
            //     poseEstimate = ll.getFilteredPose(this.associatedDrivetrain.getGyroHeading());
            // }

            Pose2d pose = this.associatedDrivetrain.getPose();
            Rotation2d gyroHeading = this.associatedDrivetrain.getGyroHeading();
            ChassisSpeeds speeds = this.associatedDrivetrain.getMeasuredSpeeds();

            Optional<Pose2d> rawResult = ll.getRawEstimatedPose();
            Optional<Pose2d> megaTagResult = ll.getMegaTag2EstimatedPose(gyroHeading, speeds);
            Optional<Pose2d> stablePose = ll.getStableEstimatedPose(pose, gyroHeading, speeds);

            if (rawResult.isPresent())
            {
                this.llRawPosePublisher.set(rawResult.get());
            }
            if (megaTagResult.isPresent())
            {
                this.llMegaTagPosePublisher.set(megaTagResult.get());
            }
            if (stablePose.isPresent())
            {
                this.poseEstimator.addVisionMeasurement(stablePose.get(), initialTime - ll.getLatencyInSeconds());
                this.llStablePosePublisher.set(stablePose.get());
            }
        }
    }

    public void reset(Pose2d initialPose)
    {
        this.poseEstimator.resetPose(initialPose);
        for (Limelight ll : this.visionContributors)
        {
            ll.resetPose(initialPose);
        }
    }

    public void addVisionContributor(Limelight contributor)
    {
        this.visionContributors.add(contributor);
    }

    private class LocalOdometryThread
    {
        public OdometryBuffer buffer;

        private Notifier threadManager = new Notifier(this::run);

        public LocalOdometryThread()
        {
            // 250 Hz - Odometry Thread (Constants.Odometry.UPDATE_FREQ)    75 Hz - Main Loop (Constants.kMainUpdatePeriod)
            // 1 / 250 Hz = 0.004 s                                         1 / 75 Hz = 0.013 s
            // 250 Hz * 0.013 s ~= 5 samples per cycle (WRONG, IT HAS 15 samples ON AVERAGE)
            this.buffer = new OdometryBuffer(30);

            // Setup the WPILIB Notifier
            this.threadManager.setName("LocalOdometryThread");
        }
        
        private void run() 
        {
            // Record updates in local odometry and send to buffer.
            double timestamp = Timer.getFPGATimestamp();
            this.buffer.add(
                associatedDrivetrain.m_frontLeft.getMeasuredDrivePosition(),
                associatedDrivetrain.m_frontLeft.getMeasuredAzimuthRotationInDeg(),
                associatedDrivetrain.m_frontRight.getMeasuredDrivePosition(),
                associatedDrivetrain.m_frontRight.getMeasuredAzimuthRotationInDeg(),
                associatedDrivetrain.m_backLeft.getMeasuredDrivePosition(),
                associatedDrivetrain.m_backLeft.getMeasuredAzimuthRotationInDeg(),
                associatedDrivetrain.m_backRight.getMeasuredDrivePosition(),
                associatedDrivetrain.m_backRight.getMeasuredAzimuthRotationInDeg(),
                Math.toDegrees(associatedDrivetrain.getGyroHeadingInRad()),
                timestamp
            );
        }

        public void start()
        {
            // associatedDrivetrain.gyroHeading.setUpdateFrequency(Constants.Odometry.kUpdateFreq);
            // Update 250Hz
            threadManager.startPeriodic(1.0 / 250);
        }

        @SuppressWarnings("unused")
        public void stop()
        {
            this.threadManager.stop();
        }
    }
}
