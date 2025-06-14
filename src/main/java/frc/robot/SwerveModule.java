package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SwerveModule {
	// Constants of physical Robot]\[]
	public static final double kAzimuthGearing = 150 / 7;
	public static final double kDriveGearing = 6.12f;
	public static final double kDriveCircumference = 0.31919f;

	/**
	 * Max acceleration in M/s of a module.
	 */
	public static final double kMaxAcceleration = 5 * 2;

	private SparkMax AzimuthMotor;

	private CANcoder AbsoluteEncoder;

	private SparkMax DriveMotor;

	private ProfiledPIDController AzimuthPID = new ProfiledPIDController(0.08, 0, 0.0005, new TrapezoidProfile.Constraints(12 * 180, 15 * 180));

	// Voltage to meters/second
	private double VoltageToMPS;

	private SlewRateLimiter DriveRateLimiter = new SlewRateLimiter(kMaxAcceleration);

	private SwerveModuleState lastDesiredState = new SwerveModuleState();

	public SwerveModule(int azimuthMotorDeviceId, int driveMotorDeviceId, int encoderDeviceId, double voltageToMPS) {

		this.VoltageToMPS = voltageToMPS;
		this.AzimuthMotor = new SparkMax(azimuthMotorDeviceId, MotorType.kBrushless);
		SparkConfigurations.ApplyConfigPersistNoReset(this.AzimuthMotor, SparkConfigurations.BreakMode);

		this.AzimuthPID.enableContinuousInput(0, 360);
		this.AzimuthPID.setTolerance(2);

		this.AbsoluteEncoder = new CANcoder(encoderDeviceId);
		// Update at 250Hz
		this.AbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(250);

		this.DriveMotor = new SparkMax(driveMotorDeviceId, MotorType.kBrushless);
		SparkConfigurations.ApplyConfigPersistNoReset(this.DriveMotor, SparkConfigurations.BreakMode);
	}

	public double getMeasuredAzimuthRotationInDeg()
	{
		return this.AbsoluteEncoder.getAbsolutePosition().getValue().in(Units.Degrees);
	}

	public Rotation2d getAzimuthRotation() {
		return Rotation2d.fromDegrees(this.getMeasuredAzimuthRotationInDeg());
	}

	/**
	 * @return Velocity in meters per minute.
	 */
	public double getDriveVelocity() {
		return this.DriveMotor.getEncoder().getVelocity() / kDriveGearing * kDriveCircumference;
	}

	public double getMeasuredDrivePosition()
	{
		return this.DriveMotor.getEncoder().getPosition() / kDriveGearing * kDriveCircumference;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(this.getMeasuredDrivePosition(), this.getAzimuthRotation());
	}

	public void Stop() {
		this.AzimuthMotor.stopMotor();
		this.DriveMotor.stopMotor();
		this.DriveRateLimiter.reset(0);

		final Rotation2d azimuthRotation = this.getAzimuthRotation();

		this.lastDesiredState = new SwerveModuleState(0, azimuthRotation);
		this.AzimuthPID.reset(azimuthRotation.getDegrees());
	}

	public void setDesiredState(SwerveModuleState state) 
	{
		this.update();
		
		state.speedMetersPerSecond = this.DriveRateLimiter.calculate(state.speedMetersPerSecond);
		this.lastDesiredState = state;
		
		double driveVoltage = state.speedMetersPerSecond * this.VoltageToMPS;

		this.DriveMotor.setVoltage(driveVoltage);

		this.update();
	}

	public void update() {
		double error = this.AzimuthPID.calculate(this.getAzimuthRotation().getDegrees(), this.lastDesiredState.angle.getDegrees());

		double controlVoltage = error * 0.95137420707;

		this.AzimuthMotor.setVoltage(controlVoltage);

		// TODO: Update DRIVE
	}

	public SwerveModuleState getMeasuredState() {
		return new SwerveModuleState(this.getDriveVelocity() / 60, this.getAzimuthRotation());
	}

	public SwerveModuleState getDesiredState() {
		return lastDesiredState;
	}

	public double getSpeed() {
		return this.DriveMotor.getEncoder().getVelocity() / 60 / kDriveGearing * kDriveCircumference;
	}
}