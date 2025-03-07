// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;

//import java.io.Console;
//import java.lang.ModuleLayer.Controller;

//import org.opencv.core.Mat;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Arm.Arm;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Elevator.Elevator;
import frc.robot.Elevator.ElevatorPosition;
import frc.robot.Limelight.Limelight;

public class Robot extends TimedRobot {

	private final Drivetrain m_swerve = new Drivetrain();
	private final XboxController m_controller = new XboxController(0);
	//private final Controller m_controllers;
	private final Elevator elevator = new Elevator(40, 8, 9);
	private final Arm m_arm = new Arm(50, 52, 51);

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(4);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

	private SendableChooser<Command> autoChooser;

	private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
	private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
	public ChassisSpeeds speeds;

	@Override
	public void robotInit() {
		this.m_swerve.resetPose(FieldPositions.Base);

		NamedCommands.registerCommand("RaiseElevatorL1", this.elevator.RaiseToL1);
		NamedCommands.registerCommand("RaiseElevatorL2", this.elevator.RaiseToL2);
		NamedCommands.registerCommand("RaiseElevatorL3", this.elevator.RaiseToL3);
		NamedCommands.registerCommand("RaiseElevatorL4", this.elevator.RaiseToL4);
		NamedCommands.registerCommand("RaiseElevatorStow", this.elevator.RaiseToStow);
		NamedCommands.registerCommand("Score", this.m_arm.ScoreAlgaeCommand);
		NamedCommands.registerCommand("Intake", this.m_arm.IntakeAlgaeCommand);
		NamedCommands.registerCommand("IntakeCoral", this.m_arm.IntakeCoralCommand);
		NamedCommands.registerCommand("ScoreCoral", this.m_arm.ScoreCoralCommand);

		this.autoChooser = AutoBuilder.buildAutoChooser();
		// this.autoChooser.setDefaultOption("None", Commands.none());
		this.autoChooser.setDefaultOption("Odometry TEST A", new PathPlannerAuto("Odometry TEST A"));

		SmartDashboard.putData("Auto Chooser", this.autoChooser);

		Limelight.registerDevice("limelight");
	}

	@Override

	public void robotPeriodic() {
		SmartDashboard.putNumber("ElevatorHeight", this.elevator.getHeight());

		if (m_controller.getStartButtonPressed()) {
			// this.m_swerve.resetPose(FieldPositions.Base);
			if (this.m_swerve.resetPoseWithLimelight())
			{
				// Rumble the controller if pose was reset to limelight estimate
				Commands
					.runEnd(() -> m_controller.setRumble(RumbleType.kBothRumble, 1), () -> m_controller.setRumble(RumbleType.kBothRumble, 0))
					.withTimeout(0.2)
					.schedule();
			}
		}
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
		// This ensures the robot does not continue to move again after re-enabling!
		// Clears the drive rate limiter and etc.
		this.m_swerve.Stop();
		this.elevator.stop();
		this.m_arm.stop();
	}

	@Override
	public void autonomousPeriodic() {

		m_swerve.updateOdometry();

	}

	@Override
	public void autonomousInit() {

		autoChooser.getSelected()
			.beforeStarting(Commands.runOnce(() -> System.out.println("We are starting!")))
			.andThen(Commands.runOnce(() -> m_swerve.Stop()))
			.andThen(Commands.runOnce(() -> System.out.println("We are done!")))
			.schedule();
	}

	@Override

	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopPeriodic() {
		driveWithJoystick(true);
	}

	private double curveJoystick(double joystickInput) 
	{
		return Math.copySign(Math.pow(MathUtil.applyDeadband(joystickInput, 0.2), 2), joystickInput);
	}

	private void driveWithJoystick(boolean fieldRelative) {
		double overclock = 3;
		// boolean overclocked;

		if (m_controller.getPOV() == 270) {

			overclock = Drivetrain.kMaxSpeed;
		}

		if (m_controller.getPOV() == 45) {
			// this.elevator.setPositions(ElevatorPosition.L1);
		}

		if (m_controller.getPOV() == 0) {
			this.elevator.setPositions(ElevatorPosition.Stowed);
			this.m_arm.setTargetRotation(Rotation2d.fromDegrees(0));
		}

		if (m_controller.getPOV() == 180) {
			// this.elevator.setPositions(ElevatorPosition.L2);
			// this.m_arm.setTargetRotation(Rotation2d.fromDegrees(185));
		}

		if (m_controller.getPOV() == 90) {
			// this.elevator.setPositions(ElevatorPosition.L3);
			// this.m_arm.setTargetRotation(Rotation2d.fromDegrees(90));
		}

		if (m_controller.getPOV() == 270) {
			// this.elevator.setPositions(ElevatorPosition.L3);
			this.m_arm.setTargetRotation(Rotation2d.fromDegrees(115));




			
		}

		if (m_controller.getLeftBumper()) {
			this.elevator.setPositions(ElevatorPosition.Stowed);
		} 
		else if (m_controller.getRightBumper()) {
			this.elevator.setPositions(ElevatorPosition.L4);
		}

		if (m_controller.getAButton()) {
			//algae score
			m_arm.scoreAlgae();
			m_arm.intakeCoral();
		} else if (!m_controller.getAButton()){
			m_arm.stopCoral();
			m_arm.stopAlgae();
		}
		
		if (m_controller.getYButton()) {
			m_arm.intakeAlgae();
			m_arm.scoreCoral();
		} else if (!m_controller.getYButton()){
			m_arm.stopCoral();
			m_arm.stopAlgae();
		}

		if(m_controller.getXButton()) {
			m_arm.intakeAlgae();
		} else if (m_controller.getBButton()) {
			m_arm.scoreAlgae();

		}


		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed = xFilter.calculate(curveJoystick(m_controller.getLeftY())) * overclock;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = yFilter.calculate(curveJoystick(m_controller.getLeftX())) * overclock;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		// final var rot =
		// -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(),
		// 0.1)) * Drivetrain.kMaxAngularSpeed;

		double rot = curveJoystick(-m_controller.getRightX()) * overclock;

		ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

		m_swerve.Drive(speeds, fieldRelative);
	}
}
