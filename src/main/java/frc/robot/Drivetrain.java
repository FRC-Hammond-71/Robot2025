// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  private final Field2d m_field = new Field2d();
  

  public static final double kMaxSpeed = 3; //in mps
  public static final double kMaxAngularSpeed = Math.PI/2; 

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(14, 15, 20);
  private final SwerveModule m_frontRight = new SwerveModule(16, 17, 21);
  private final SwerveModule m_backLeft = new SwerveModule(12, 13, 22);
  private final SwerveModule m_backRight = new SwerveModule(10, 11, 23);
  private boolean isChangingRotationLast = true;

  // Lower when we add simple feed forward!!!!!!
  private final PIDController m_headingPID = new PIDController(1.2, 0, 0);
  // private final ProfiledPIDController m_headingPID = new
  // ProfiledPIDController(0.5,0, 0, new TrapezoidProfile.Constraints(Math.PI,
  // Math.PI / 4));[]\
  
  // private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose2d.struct).publish();


  public final StructArrayPublisher<SwerveModuleState> ActualSwervePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("ActualStates", SwerveModuleState.struct).publish();
  public final StructArrayPublisher<SwerveModuleState> DesiredSwervePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();

    public final StructPublisher<ChassisSpeeds> RelativeSpeedsPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("RelativeSpeeds", ChassisSpeeds.struct).publish();
    public final StructPublisher<ChassisSpeeds> DesiredRelativeSpeedsPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("DesiredRelativeSpeeds", ChassisSpeeds.struct).publish();


  // public final StructArrayPublisher<ChassisSpeeds> ChassisSpeeds =
  // NetworkTableInstance.getDefault()
  // .getStructArrayTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
      m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  public void resetPose(Pose2d initialPose) {
    m_gyro.reset();
    m_gyro.zeroYaw();
    m_headingPID.reset();
    m_headingPID.setSetpoint(initialPose.getRotation().getRadians());
    m_odometry.resetPose(initialPose);
    // isChangingRotationLast = true;
  }

  public Drivetrain() {
    SmartDashboard.putData("Field", m_field);

    this.m_headingPID.setTolerance(4 * (Math.PI / 180));

    setupPathPlanner();
  }

  private void setupPathPlanner()
  {
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
      this::getPose,
      this::resetPose,
      this::getRelativeSpeeds,
      (speeds) -> this.Drive(speeds, false),
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants

      ),
      config,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );

    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> { m_field.setRobotPose(pose); });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> { m_field.getObject("target pose").setPose(pose); });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> { m_field.getObject("path").setPoses(poses); });
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
  public void Drive(ChassisSpeeds speeds, boolean fieldRelative) {

    // Our strategy to enable rotation is the following:
    // - Control is not commanding a rotation, keep the current angle!
    // - Control IS commanding a rotation from speeds.omegaRadiansPerSecond, do not
    // use PID.
    final boolean isUserChangingRotation = speeds.omegaRadiansPerSecond != 0;

    SmartDashboard.putBoolean("ChangingRot", isUserChangingRotation);

    if (isUserChangingRotation) {
      // We do not modify this input, this enables an instant-response to fine-control
      // unlike a PID loop.
      // speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;

      isChangingRotationLast = true;

      SmartDashboard.putNumber("PIDEffort", 0);

    } else {
      if (isChangingRotationLast) {
        // Last loop we were rotating from speeds, now we are not! Update our PID
        // setpoint to current angle ONCE.
        m_headingPID.setSetpoint(getGyroHeading().getRadians());
      }



      SmartDashboard.putNumber("PIDSetpoint", Math.toDegrees(m_headingPID.getSetpoint()));
 

      // No input, utilize PID to keep the heading!
      speeds.omegaRadiansPerSecond = m_headingPID.calculate(getGyroHeading().getRadians());

      SmartDashboard.putNumber("PIDEffort", Math.toDegrees(speeds.omegaRadiansPerSecond));

      // This will prevent setSetpoint running every cycle we don't have an input
      // (Bad! What if we get hit and rotate?)
      // the PID setPoint would be "corrupted".
      isChangingRotationLast = false;
    }

    // speeds.omegaRadiansPerSecond = 0;

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

    // SwerveModuleState[] swerveModuleStates =
    // m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    this.updateOdometry();
    
    ActualSwervePublisher.set(this.getMeasuredModulesStates());
    DesiredSwervePublisher.set(swerveModuleStates);    
    this.publisher.set(this.getPose());
    this.RelativeSpeedsPublisher.set(this.getRelativeSpeeds());
    // this.DesiredRelativeSpeedsPublisher.set(speeds);

  }

  public SwerveModuleState[] getMeasuredModulesStates()
  {
    SwerveModuleState[] measuredSwerveStates = {
      m_frontLeft.getMeasuredState(),
      m_frontRight.getMeasuredState(),
      m_backLeft.getMeasuredState(),
      m_backRight.getMeasuredState() 
    };

    return measuredSwerveStates;
  }

  public ChassisSpeeds getDesiredSpeeds()
  {
    SwerveModuleState[] desiredSwerveStates = {
      m_frontLeft.getDesiredState(),
      m_frontRight.getDesiredState(),
      m_backLeft.getDesiredState(),
      m_backRight.getDesiredState()
    };

    return this.m_kinematics.toChassisSpeeds(desiredSwerveStates);
  }

  public void Stop() {
    m_backLeft.Stop();
    m_backRight.Stop();
    m_frontLeft.Stop();
    m_frontRight.Stop();
    this.m_headingPID.setSetpoint(this.getPose().getRotation().getRadians());
  }

  public ChassisSpeeds getRelativeSpeeds()
  {
    return this.m_kinematics.toChassisSpeeds(this.getMeasuredModulesStates());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        this.getGyroHeading(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  private Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  } 

  public Pose2d getPose()
  {
    // return this.m_field.getRobotPose();
    return this.m_odometry.getPoseMeters();
  }

  public void dashboardPrint() {
    SmartDashboard.putNumber("FR Rotation", m_frontRight.getAzimuthRotation().getDegrees());
    SmartDashboard.putNumber("FL Rotation", m_frontLeft.getAzimuthRotation().getDegrees());
    SmartDashboard.putNumber("BR Rotation", m_backRight.getAzimuthRotation().getDegrees());
    SmartDashboard.putNumber("BL Rotation", m_backLeft.getAzimuthRotation().getDegrees());
    SmartDashboard.putNumber("Heading", this.getPose().getRotation().getDegrees());
    // SmartDashboard.putString("State fr", m_frontRight.getState().toString());
    // SmartDashboard.putString("State fl", m_frontLeft.getState().toString());
    // SmartDashboard.putString("State br", m_backRight.getState().toString());
    // SmartDashboard.putString("State bl", m_frontRight.getState().toString());

  }

}
