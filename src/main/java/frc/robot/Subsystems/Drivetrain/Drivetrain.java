// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.ReefWaypointGenerator;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Attractors.PointAttractor;
import frc.robot.Attractors.Controllers.FaceController;
import frc.robot.Limelight.Limelight;
import frc.robot.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.ReefWaypointGenerator.ReefEdgeWaypoints;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
	public final Field2d m_field = new Field2d();

	public static final double kMaxSpeed = 4.3; // in MP/s
	public static final double kMaxAngularSpeed = Math.PI * 1.5;

	private final Translation2d m_frontLeftLocation = new Translation2d(0.2635, 0.2635);
	private final Translation2d m_frontRightLocation = new Translation2d(0.2635, -0.2635);
	private final Translation2d m_backLeftLocation = new Translation2d(-0.2635, 0.2635);
	private final Translation2d m_backRightLocation = new Translation2d(-0.2635, -0.2635);

	public final SwerveModule m_frontLeft = new SwerveModule(14, 15, 20, 2.236);
	public final SwerveModule m_frontRight = new SwerveModule(16, 17, 21, 2.38472);
	public final SwerveModule m_backLeft = new SwerveModule(12, 13, 22, 2.3504);
	public final SwerveModule m_backRight = new SwerveModule(10, 11, 23, 2.28488);
	private boolean isChangingRotationLast = true;
	

	// Lower when we add simple feed forward!!!!!!
	private final PIDController m_headingPID = new PIDController(1.7, 0, 0.005);

	public final Pigeon2 m_gyro = new Pigeon2(30);

	private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("rPose", Pose2d.struct).publish();

	private StructPublisher<Pose2d> llRawPosePublisher = NetworkTableInstance.getDefault()
		.getStructTopic("ll_RawPose", Pose2d.struct).publish();
		
	private StructPublisher<Pose2d> llMegaTagPosePublisher = NetworkTableInstance.getDefault()
		.getStructTopic("ll_MegaTagPose", Pose2d.struct).publish();

	private StructPublisher<Pose2d> llStablePosePublisher = NetworkTableInstance.getDefault()
		.getStructTopic("ll_StablePose", Pose2d.struct).publish();

	public final StructArrayPublisher<SwerveModuleState> measuredSwervePublisher = NetworkTableInstance.getDefault()
			.getStructArrayTopic("ActualStates", SwerveModuleState.struct).publish();
	public final StructArrayPublisher<SwerveModuleState> desiredSwervePublisher = NetworkTableInstance.getDefault()
			.getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();

	public final StructPublisher<ChassisSpeeds> relativeSpeedsPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("RelativeSpeeds", ChassisSpeeds.struct).publish();
	public final StructPublisher<ChassisSpeeds> desiredRelativeSpeedsPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("DesiredRelativeSpeeds", ChassisSpeeds.struct).publish();

	public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
			m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	public boolean resetPoseWithLimelight()
	{
		Optional<Pose2d> es = Limelight.useDevice("limelight").getRawEstimatedPose();

		if (es.isPresent())
		{
			this.resetPose(es.get());
			return true;
		}
		return false;
	}

	public void resetPose(Pose2d initialPose) {
		this.m_gyro.reset();
		this.m_gyro.setYaw(initialPose.getRotation().getDegrees());
		this.m_headingPID.reset();
		this.m_headingPID.setSetpoint(initialPose.getRotation().getRadians());
		Limelight.useDevice("limelight").resetPose(initialPose);
		this.posePublisher.set(initialPose);
		this.m_field.setRobotPose(initialPose);
		this.odometry.reset(initialPose);
		this.isChangingRotationLast = true;
	}

	public Odometry odometry;

	public Drivetrain() {
		SmartDashboard.putData("Field", m_field);

		this.m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
		this.m_headingPID.setTolerance(3 * (Math.PI / 180));

		// Update at 250Hz
		this.m_gyro.getYaw().setUpdateFrequency(250);

		this.odometry = new Odometry(this);

		final List<ReefEdgeWaypoints> waypoints = ReefWaypointGenerator.generateHexagonPoses();
		for (int e = 0; e < waypoints.size(); e++)
		{
			this.m_field.getObject(String.format("Edge %d L4 Left", e)).setPose(waypoints.get(e).LeftL4Coral);
			this.m_field.getObject(String.format("Edge %d L4 Right", e)).setPose(waypoints.get(e).RightL4Coral);
		}

		setupPathPlanner();
	}

	private void setupPathPlanner() {
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
			return;
		}
		AutoBuilder.configure(
			this::getPose,
			(pose) -> {
				// this.resetPose(pose);	
			},
			this::getRelativeSpeeds,
			(speeds) -> this.Drive(speeds, false),
			new PPHolonomicDriveController(
					new PIDConstants(3.3, 0.0, 0), // Translation PID constants
					new PIDConstants(Math.PI * 1.6, 0.0, 0) // Rotation PID constants
			),
			config,
			() -> {
				// Boolean supplier that controls when the path will be mirrored for the red
				// alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
			this);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	public void Drive(ChassisSpeeds speeds, boolean fieldRelative) 
	{
		final boolean isUserChangingRotation = speeds.omegaRadiansPerSecond != 0;
		if (isUserChangingRotation) 
		{
			// We do not modify this input, this enables an instant-response to fine-control
			// unlike a PID loop.
			// speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;

			isChangingRotationLast = true;
		} 
		else 
		{
			if (isChangingRotationLast) 
			{
				// TODO: Scale yaw velocity up or down depending on latency.
				m_headingPID.setSetpoint(this.getPose().getRotation().getRadians());
			}

			// No input, utilize PID to keep the heading!
			double headingKeepValue = m_headingPID.calculate(this.getPose().getRotation().getRadians());
			if (Math.abs(headingKeepValue) > 4 * (Math.PI / 180))
			{
				speeds.omegaRadiansPerSecond = headingKeepValue;
			}
			isChangingRotationLast = false;
		}

		// We are still going to clamp the rotation speed!
		speeds.omegaRadiansPerSecond = MathUtil.clamp(speeds.omegaRadiansPerSecond, -kMaxAngularSpeed, kMaxAngularSpeed);
		speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -kMaxSpeed, kMaxSpeed);
		speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -kMaxSpeed, kMaxSpeed);

		speeds = ChassisSpeeds.discretize(speeds, Robot.kDefaultPeriod);
		speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getPose().getRotation()) : speeds;

		SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

		swerveModuleStates[0].optimize(this.m_frontLeft.getAzimuthRotation());
		swerveModuleStates[1].optimize(this.m_frontRight.getAzimuthRotation());
		swerveModuleStates[2].optimize(this.m_backLeft.getAzimuthRotation());
		swerveModuleStates[3].optimize(this.m_backRight.getAzimuthRotation());

		swerveModuleStates[0].speedMetersPerSecond *= swerveModuleStates[0].angle
				.minus(this.m_frontLeft.getAzimuthRotation()).getCos();
		swerveModuleStates[1].speedMetersPerSecond *= swerveModuleStates[1].angle
				.minus(this.m_frontRight.getAzimuthRotation()).getCos();
		swerveModuleStates[2].speedMetersPerSecond *= swerveModuleStates[2].angle
				.minus(this.m_backLeft.getAzimuthRotation()).getCos();
		swerveModuleStates[3].speedMetersPerSecond *= swerveModuleStates[3].angle
				.minus(this.m_backRight.getAzimuthRotation()).getCos();

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);

		this.measuredSwervePublisher.set(this.getMeasuredModulesStates());
		this.desiredSwervePublisher.set(swerveModuleStates);
		this.relativeSpeedsPublisher.set(this.getRelativeSpeeds());
	}

	@Override
	public void periodic() 
	{
		// this.updateOdometry();
		this.odometry.update();
		this.posePublisher.set(this.odometry.getEstimatedPose());
		this.dashboardPrint();

        if (DriverStation.isDisabled()) return;

		this.m_frontLeft.update();
		this.m_frontRight.update();
		this.m_backLeft.update();
		this.m_backRight.update();
	}

	public SwerveModuleState[] getMeasuredModulesStates() {
		SwerveModuleState[] measuredSwerveStates = {
			m_frontLeft.getMeasuredState(),
			m_frontRight.getMeasuredState(),
			m_backLeft.getMeasuredState(),
			m_backRight.getMeasuredState()
		};
		return measuredSwerveStates;
	}

	public ChassisSpeeds getDesiredSpeeds() {
		SwerveModuleState[] desiredSwerveStates = {
				m_frontLeft.getDesiredState(),
				m_frontRight.getDesiredState(),
				m_backLeft.getDesiredState(),
				m_backRight.getDesiredState()
		};
		return this.m_kinematics.toChassisSpeeds(desiredSwerveStates);
	}

	public ChassisSpeeds getMeasuredSpeeds() {
		return this.m_kinematics.toChassisSpeeds(this.getMeasuredModulesStates());
	}

	public SwerveModulePosition[] getModulePositions()
	{
		return new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_backLeft.getPosition(),
			m_backRight.getPosition()
		};
	}

	public void resetGyro(Rotation2d rot)
	{
		this.resetPose(new Pose2d(this.getPose().getTranslation(), rot));
	}

	public void Stop() {
		m_backLeft.Stop();
		m_backRight.Stop();
		m_frontLeft.Stop();
		m_frontRight.Stop();
		this.m_headingPID.setSetpoint(this.getPose().getRotation().getRadians());
	}

	public ChassisSpeeds getRelativeSpeeds() {
		return this.m_kinematics.toChassisSpeeds(this.getMeasuredModulesStates());
	}


	/** Updates the field relative position of the robot. */
	public void updateOdometry() 
	{
		// this.m_odometry.update(this.getGyroHeading(), new SwerveModulePosition[] {
		// 	m_frontLeft.getPosition(),
		// 	m_frontRight.getPosition(),
		// 	m_backLeft.getPosition(),
		// 	m_backRight.getPosition()
		// });

		// Pose2d estimatedPose = this.getPose();
		// Rotation2d gyroHeading = this.getGyroHeading();
		// ChassisSpeeds speeds = this.getMeasuredSpeeds();
		// Limelight limelight = Limelight.useDevice("limelight");

		// boolean useVision = SmartDashboard.getBoolean("Drivetrain/useVision", true);

		// Optional<Pose2d> rawResult = limelight.getRawEstimatedPose();
		// Optional<Pose2d> megaTagResult = limelight.getMegaTag2EstimatedPose(gyroHeading, speeds);
		// Optional<Pose2d> stablePose = limelight.getStableEstimatedPose(estimatedPose, gyroHeading, speeds);

		// if (rawResult.isPresent())
		// {
		// 	this.llRawPosePublisher.set(rawResult.get());
		// }
		// if (megaTagResult.isPresent())
		// {
		// 	this.llMegaTagPosePublisher.set(megaTagResult.get());
		// }
		// if (stablePose.isPresent())
		// {
		// 	Pose2d newStablePose = new Pose2d(stablePose.get().getTranslation(), estimatedPose.getRotation());

		// 	// Only contribute the stablePose to Pose Estimation!
		// 	if (useVision)
		// 	{
		// 		this.m_odometry.addVisionMeasurement(newStablePose, Timer.getFPGATimestamp() - limelight.getLatencyInSeconds());
		// 	}
		// 	this.llStablePosePublisher.set(newStablePose);
		// }
		// this.posePublisher.set(this.getPose());
	}

	public double getGyroHeadingInRad()
	{
		return MathUtil.inputModulus(this.m_gyro.getYaw().getValue().in(Units.Radians), -Math.PI, Math.PI);
	}

	public Rotation2d getGyroHeading() {
		// Gyro at zero should be facing the enemy side!
		return Rotation2d.fromRadians(this.getGyroHeadingInRad());
	}

	/**
	 * @return Yaw-rate/speed per second.
	 */
	private Rotation2d getGyroVelocity()
	{
		return Rotation2d.fromDegrees(this.m_gyro.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond));
	}

	public Pose2d getPose() {
		if (Robot.isSimulation())
		{
			return this.m_field.getRobotPose();
		}
		// return this.m_odometry.getEstimatedPosition();
		return this.odometry.getEstimatedPose();
	}

	private void dashboardPrintModuleSpeeds(String name, SwerveModule module) {
		SmartDashboard.putNumber(String.format("Drivetrain/Module %s Desired-Speed", name), module.getDesiredState().speedMetersPerSecond);
		SmartDashboard.putNumber(String.format("Drivetrain/%s Measured-Speed", name),module.getMeasuredState().speedMetersPerSecond);
	}

	public void dashboardPrintSwerveDrive() {
		SmartDashboard.putData("Swerve Drivetrain", new Sendable() {
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				
				builder.addDoubleProperty("Front Left Heading", () -> m_frontLeft.getAzimuthRotation().getDegrees(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getDriveVelocity(), null);

				builder.addDoubleProperty("Front Right Heading", () -> m_frontRight.getAzimuthRotation().getDegrees(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getDriveVelocity(), null);

				builder.addDoubleProperty("Back Left Heading", () -> m_backLeft.getAzimuthRotation().getDegrees(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getDriveVelocity(), null);

				builder.addDoubleProperty("Back Right Heading", () -> m_backRight.getAzimuthRotation().getDegrees(), null);
				builder.addDoubleProperty("Back Right Velocity", () ->  m_backRight.getDriveVelocity(), null);

				builder.addDoubleProperty("Robot Heading", () -> getGyroHeading().getDegrees(), null);				
			}
		});
	}	

	public void dashboardPrint() {
		SmartDashboard.putNumber("Drivetrain/Heading", this.getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("Drivetrain/GyroHeading", this.getGyroHeading().getDegrees());

		dashboardPrintModuleSpeeds("FL", m_frontLeft);
		dashboardPrintModuleSpeeds("FR", m_frontRight);
		dashboardPrintModuleSpeeds("BL", m_backLeft);
		dashboardPrintModuleSpeeds("BR", m_backRight);
		
	}
}
