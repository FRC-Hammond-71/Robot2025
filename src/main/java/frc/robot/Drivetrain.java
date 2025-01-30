// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 0.5; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI / 4; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(10,11);
  private final SwerveModule m_frontRight = new SwerveModule(12,13);
  private final SwerveModule m_backLeft = new SwerveModule(14,15);
  private final SwerveModule m_backRight = new SwerveModule(16,17);

  private final PIDController m_headingPID = new PIDController(0.25, 0, 0);
  // private final ProfiledPIDController m_headingPID = new ProfiledPIDController(0.5,0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI / 4));
  //private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  
  private final SwerveDriveKinematics m_kinematics =
	  new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
	  new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_backLeft.getPosition(),
			m_backRight.getPosition()
		  });

  public void GyroReset() {
	  m_gyro.reset();
    m_gyro.zeroYaw();
  }


  public Drivetrain()
  {
    this.m_gyro.resetDisplacement();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {

    this.m_headingPID.enableContinuousInput(0, Math.PI * 2);
    this.m_headingPID.setTolerance(5 * (Math.PI / 180));

    // final ChassisSpeeds m_cSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 45);
    
    final double kpOutput = m_headingPID.calculate(getGyroHeading().getRadians(), rot);
    
    SmartDashboard.putNumber("PID-Calculated Output", kpOutput);

    // ChassisSpeeds TargetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, kpOutput, this.getGyroHeading());

    // SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, kpOutput, m_gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, kpOutput), periodSeconds));
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, kpOutput));
    
    swerveModuleStates[0].optimize(this.m_frontLeft.getAzimuthRotation());
    swerveModuleStates[1].optimize(this.m_frontRight.getAzimuthRotation());
    swerveModuleStates[2].optimize(this.m_backRight.getAzimuthRotation());
    swerveModuleStates[3].optimize(this.m_backLeft.getAzimuthRotation());

    swerveModuleStates[0].speedMetersPerSecond*=swerveModuleStates[0].angle.minus(this.m_frontLeft.getAzimuthRotation()).getCos();
    swerveModuleStates[1].speedMetersPerSecond*=swerveModuleStates[1].angle.minus(this.m_frontRight.getAzimuthRotation()).getCos();
    swerveModuleStates[2].speedMetersPerSecond*=swerveModuleStates[2].angle.minus(this.m_backRight.getAzimuthRotation()).getCos();
    swerveModuleStates[3].speedMetersPerSecond*=swerveModuleStates[3].angle.minus(this.m_backLeft.getAzimuthRotation()).getCos();

    

    // SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_backLeft.setDesiredState(swerveModuleStates[3]);

    // Output order is Front-Left, Front-Right, Back-Right, Back-Left
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
	m_odometry.update(
		m_gyro.getRotation2d(), new SwerveModulePosition[] {
		  m_frontLeft.getPosition(),
		  m_frontRight.getPosition(),
		  m_backLeft.getPosition(),
		  m_backRight.getPosition()
		});
  }

  public Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void dashboardPrint() {
	SmartDashboard.putNumber("FR Rotation", m_frontRight.getAzimuthRotation().getDegrees());
	SmartDashboard.putNumber("FL Rotation", m_frontLeft.getAzimuthRotation().getDegrees());
	SmartDashboard.putNumber("BR Rotation", m_backRight.getAzimuthRotation().getDegrees());
	SmartDashboard.putNumber("BL Rotation", m_backLeft.getAzimuthRotation().getDegrees());
  SmartDashboard.putNumber("Gyro Heading", this.getGyroHeading().getDegrees());
	// SmartDashboard.putString("State fr", m_frontRight.getState().toString());
	// SmartDashboard.putString("State fl", m_frontLeft.getState().toString());
	// SmartDashboard.putString("State br", m_backRight.getState().toString());
	// SmartDashboard.putString("State bl", m_frontRight.getState().toString());

  }



}
