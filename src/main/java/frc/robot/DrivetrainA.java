// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import java.util.Arrays;
// import java.util.List;
// import java.util.Optional;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;
// import com.studica.frc.AHRS.NavXUpdateRate;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// // import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructArrayPublisher;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// // import edu.wpi.first.math.trajectory.TrapezoidProfile;
// // import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
// // import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Attractors.Attractor;
// import frc.robot.Attractors.LineAttractor;
// import frc.robot.Attractors.PointAttractor;
// import frc.robot.Attractors.Controllers.PoseController;
// import frc.robot.Attractors.Controllers.RotationController;
// import frc.robot.LimelightHelpers.PoseEstimate;

// /** Represents a swerve drive style drivetrain. */
// public class DrivetrainA extends SubsystemBase {
// 	private final Field2d m_field = new Field2d();
// 	public static final double kMaxSpeed = 2; // 3 meters per second
// 	public static final double kMaxAngularSpeed = Math.PI / 2; // 1/2 rotation per second

// 	private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
// 	private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
// 	private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
// 	private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

// 	private final SwerveModule m_frontLeft = new SwerveModule(14, 15, 1);
// 	private final SwerveModule m_frontRight = new SwerveModule(16, 17, 2);
// 	private final SwerveModule m_backLeft = new SwerveModule(12, 13, 3);
// 	private final SwerveModule m_backRight = new SwerveModule(10, 11, 4);

// 	// Lower when we add simple feed forward!!!!!!
// 	private final PIDController m_headingPID = new PIDController(1.8, 0, 0);
// 	// private final ProfiledPIDController m_headingPID = new
// 	// ProfiledPIDController(0.5,0, 0, new TrapezoidProfile.Constraints(Math.PI,
// 	// Math.PI / 4));
// 	// private final AnalogGyro m_gyro = new AnalogGyro(0);
// 	private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

// 	private boolean wasSlipping = false;

// 	private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
// 			.getStructTopic("MyPose", Pose2d.struct).publish();

// 	public final StructArrayPublisher<SwerveModuleState> ActualSwervePublisher = NetworkTableInstance.getDefault()
// 			.getStructArrayTopic("ActualStates", SwerveModuleState.struct).publish();
// 	public final StructArrayPublisher<SwerveModuleState> DesiredSwervePublisher = NetworkTableInstance.getDefault()
// 			.getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();

// 	private StructPublisher<Pose2d> llNonFilteredPosePublisher = NetworkTableInstance.getDefault()
// 		.getStructTopic("UnfilteredPose", Pose2d.struct).publish();
// 	private StructPublisher<Pose2d> llFilteredPosePublisher = NetworkTableInstance.getDefault()
// 		.getStructTopic("UnfilteredPose", Pose2d.struct).publish();

// 	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
// 		m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

// 	private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(m_kinematics,
// 			m_gyro.getRotation2d(),
// 			new SwerveModulePosition[] {
// 					m_frontLeft.getPosition(),
// 					m_frontRight.getPosition(),
// 					m_backLeft.getPosition(),
// 					m_backRight.getPosition()
// 			}, new Pose2d(0, 0, new Rotation2d(0)));

// 	// public final StructArrayPublisher<ChassisSpeeds> ChassisSpeeds =
// 	// NetworkTableInstance.getDefault()
// 	// .getStructArrayTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();

// 	public static PoseController AlgaeNetShootingController = new PoseController(
// 		Field.Attractors.AlgaeNetShootingBlueRotation,
// 		Field.Attractors.AlgaeNetShottingBlueAttractor);

// 	// private final SwerveDriveOdometry m_odometry;

// 	// Drop-in replacement for SwerveDriveOdometry.
// 	private final SwerveDrivePoseEstimator m_postEstimator;

// 	public void GyroReset() {
// 		m_gyro.reset();
// 		m_gyro.zeroYaw();
// 	}

// 	public DrivetrainA() {
// 		SmartDashboard.putData("Field", m_field);

// 		this.m_postEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(),
// 				new SwerveModulePosition[] {
// 						m_frontLeft.getPosition(),
// 						m_frontRight.getPosition(),
// 						m_backLeft.getPosition(),
// 						m_backRight.getPosition()
// 				}, new Pose2d());

// 		// m_field.getObject("Reef
// 		// Attractors").setPoses(this.coralRotationAttractors.stream().map((a) ->
// 		// a.getPose()).toList());
// 	}

// 	@Override
// 	public void periodic() {
// 		// TODO: Change setVisionMeasurementStdDevs based on our current situation.

// 		// if ((this.m_frontLeft.isSlipping() || this.m_frontRight.isSlipping() ||
// 		// this.m_backLeft.isSlipping() || this.m_backRight.isSlipping()) &&
// 		// !wasSlipping)
// 		// {
// 		// // Our modules are slipping!!
// 		// this.m_postEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.2, 0.2,
// 		// Units.degreesToRadians(5)));
// 		// this.wasSlipping = true;
// 		// }
// 		// else if (wasSlipping)
// 		// {
// 		// this.m_postEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.9, 0.9,
// 		// 0.9));
// 		// this.wasSlipping = false;
// 		// }

// 		// Update our pose predictions every cycle
// 		this.m_postEstimator.update(this.getGyroHeading(), new SwerveModulePosition[] {
// 				m_frontLeft.getPosition(),
// 				m_frontRight.getPosition(),
// 				m_backLeft.getPosition(),
// 				m_backRight.getPosition()
// 		});
// 	}

// 	public Pose2d getPose() {
// 		return m_field.getRobotPose();
// 	}

// 	/**
// 	 * Method to drive the robot using joystick info.
// 	 *
// 	 * @param xSpeed        Speed of the robot in the x direction (forward).
// 	 * @param ySpeed        Speed of the robot in the y direction (sideways).
// 	 * @param rot           Angular rate of the robot.
// 	 * @param fieldRelative Whether the provided x and y speeds are relative to the
// 	 *                      field.
// 	 */
// 	public void Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {

// 		// this.m_headingPID.enableContinuousInput(0, Math.PI * 2);
// 		this.m_headingPID.setTolerance(2 * (Math.PI / 180));

// 		// final ChassisSpeeds m_cSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 45);

// 		// Only do this when the driver is NOT giving rotation input!
// 		if (true) {
// 			// System.out.println(m_field.getRobotPose());
// 			RotationProvider nearestAttractor = Attractor.getNearestAttractor(this.getPose().getTranslation(), this.coralRotationAttractors);

// 			// rot = nearestAttractor.attractRotation(this.getPose()).getRadians();

// 			ChassisSpeeds assistedDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(AlgaeNetShootingController.calculate(this.getPose()), this.m_field.getRobotPose().getRotation());

// 			if (assistedDesiredSpeeds.vxMetersPerSecond != 0 || assistedDesiredSpeeds.vyMetersPerSecond != 0 || assistedDesiredSpeeds.omegaRadiansPerSecond != 0)
// 			{
// 				this.m_field.setRobotPose(
// 						this.m_field.getRobotPose().plus(new Transform2d(
// 							assistedDesiredSpeeds.vxMetersPerSecond * Robot.kDefaultPeriod, 
// 							assistedDesiredSpeeds.vyMetersPerSecond * Robot.kDefaultPeriod, 
// 							Rotation2d.fromRadians(assistedDesiredSpeeds.omegaRadiansPerSecond * Robot.kDefaultPeriod)))
// 						);

// 				// Should be displayed as a line!
// 				this.m_field.getObject("Nearest Attractor").setPoses(this.getPose(), new Pose2d(AlgaeNetShootingController.getPosition(this.getPose()), Rotation2d.fromDegrees(0)));
// 			}
// 			else
// 			{
// 				// There is no way to "delete" the data we've sent so just move it out of the map view.
// 				this.m_field.getObject("Nearest Attractor").setPose(new Pose2d(-100, -100, Rotation2d.fromDegrees(0)));
// 			}

// 			this.m_field.getObject("Algae Scoring Line").setPoses(
// 					new Pose2d(Field.Attractors.AlgaeNetShottingBlueAttractor.start, Field.Attractors.AlgaeNetShootingBlueRotation), 
// 					new Pose2d(Field.Attractors.AlgaeNetShottingBlueAttractor.end, Field.Attractors.AlgaeNetShootingBlueRotation));

// 			if (nearestAttractor != null)
// 			{
// 			rot = nearestAttractor.attractRotation(this.getPose()).getRadians();

// 			this.m_field.setRobotPose(new
// 			Pose2d(this.m_field.getRobotPose().getTranslation(),
// 			Rotation2d.fromRadians(rot)));

// 			// Should be displayed as a line!
// 			this.m_field.getObject("Nearest Attractor").setPoses(this.getPose(),
// 			nearestAttractor.getPose());
// 			}
// 			else
// 			{
// 			// There is no way to "delete" the data we've sent so just move it out of the
// 			map view.
// 			this.m_field.getObject("Nearest Attractor").setPose(new Pose2d(-100, -100,
// 			Rotation2d.fromDegrees(0)));
// 			}
// 		}

// 		final double kpOutput = m_headingPID.calculate(getGyroHeading().getRadians(), rot);

// 		SmartDashboard.putNumber("PID-Calculated Output", kpOutput);
// 		// final double kpOutput = 0;

// 		ChassisSpeeds TargetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, kpOutput,
// 				this.getGyroHeading());

// 		// SmartDashboard.putData("", new Pose2d(0, 0));

// 		// m_field.setRobotPose(m_odometry.getPoseMeters());
// 		// SwerveModuleState[] swerveModuleStates =
// 		// m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(fieldRelative ?
// 		// ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, kpOutput,
// 		// m_gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, kpOutput),
// 		// periodSeconds));
// 		SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(TargetSpeeds);

// 		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

// 		swerveModuleStates[0].optimize(this.m_frontLeft.getAzimuthRotation());
// 		swerveModuleStates[1].optimize(this.m_frontRight.getAzimuthRotation());
// 		swerveModuleStates[2].optimize(this.m_backLeft.getAzimuthRotation());
// 		swerveModuleStates[3].optimize(this.m_backRight.getAzimuthRotation());

// 		swerveModuleStates[0].speedMetersPerSecond *= swerveModuleStates[0].angle
// 				.minus(this.m_frontLeft.getAzimuthRotation()).getCos();
// 		swerveModuleStates[1].speedMetersPerSecond *= swerveModuleStates[1].angle
// 				.minus(this.m_frontRight.getAzimuthRotation()).getCos();
// 		swerveModuleStates[2].speedMetersPerSecond *= swerveModuleStates[2].angle
// 				.minus(this.m_backLeft.getAzimuthRotation()).getCos();
// 		swerveModuleStates[3].speedMetersPerSecond *= swerveModuleStates[3].angle
// 				.minus(this.m_backRight.getAzimuthRotation()).getCos();

// 		// SwerveModuleState[] swerveModuleStates =
// 		// m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
// 		m_frontLeft.setDesiredState(swerveModuleStates[0]);
// 		m_frontRight.setDesiredState(swerveModuleStates[1]);
// 		m_backLeft.setDesiredState(swerveModuleStates[2]);
// 		m_backRight.setDesiredState(swerveModuleStates[3]);

// 		// "formatting"
// 		SwerveModuleState[] ActualCurrentSwerveStates = {
// 				m_frontLeft.getDesiredState(),
// 				m_frontRight.getDesiredState(),
// 				m_backLeft.getDesiredState(),
// 				m_backRight.getDesiredState() };

// 		ActualSwervePublisher.set(ActualCurrentSwerveStates);
// 		DesiredSwervePublisher.set(swerveModuleStates);

// 		// NetworkTableInstance.getDefault().getStructArrayTopic(null, null)

// 		// Output order is Front-Left, Front-Right, Back-Right, Back-Left
// 	}

// 	public ChassisSpeeds getDesiredSpeeds() {
// 		SwerveModuleState[] desiredSwerveStates = {
// 				m_frontLeft.getDesiredState(),
// 				m_frontRight.getDesiredState(),
// 				m_backLeft.getDesiredState(),
// 				m_backRight.getDesiredState()
// 		};

// 		return this.m_kinematics.toChassisSpeeds(desiredSwerveStates);
// 	}

// 	/** Updates the field relative position of the robot. */
// 	public void updateOdometry() 
// 	{
// 		// ...

// 		// Incorporate limelight (ll) into our pose estimation AFTER our usual update()
// 		// {
// 		// 	Rotation2d heading = this.getGyroHeading()	;
// 		// 	ChassisSpeeds speeds = this.getMeasuredSpeeds();

// 		// 	PoseEstimate unFilteredResult = Limelight.useDevice("Limelight A").getEstimationResult(this.getGyroHeading(), speeds);
// 		// 	Optional<Pose2d> filteredPose = Limelight.useDevice("Limelight A").getFilteredEstimatedPose(this.getGyroHeading(), speeds);

// 		// 	this.llNonFilteredPosePublisher.set(unFilteredResult.pose);
// 		// 	if (filteredPose.isPresent())
// 		// 	{
// 		// 		this.llFilteredPosePublisher.set(filteredPose.get());
// 		// 		this.m_odometry.addVisionMeasurement(filteredPose.get(), Timer.getFPGATimestamp());
// 		// 	}
// 		// }
// 	}

// 	public Rotation2d getGyroHeading() {
// 		return Rotation2d.fromDegrees(-m_gyro.getAngle());
// 	}

// 	public void dashboardPrint() {
// 		SmartDashboard.putNumber("FR Rotation", m_frontRight.getAzimuthRotation().getDegrees());
// 		SmartDashboard.putNumber("FL Rotation", m_frontLeft.getAzimuthRotation().getDegrees());
// 		SmartDashboard.putNumber("BR Rotation", m_backRight.getAzimuthRotation().getDegrees());
// 		SmartDashboard.putNumber("BL Rotation", m_backLeft.getAzimuthRotation().getDegrees());
// 		SmartDashboard.putNumber("Gyro Heading", this.getGyroHeading().getDegrees());
// 		// SmartDashboard.putString("State fr", m_frontRight.getState().toString());
// 		// SmartDashboard.putString("State fl", m_frontLeft.getState().toString());
// 		// SmartDashboard.putString("State br", m_backRight.getState().toString());
// 		// SmartDashboard.putString("State bl", m_frontRight.getState().toString());

// 	}

// }
