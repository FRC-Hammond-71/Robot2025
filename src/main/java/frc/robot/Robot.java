// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

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

	private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
	private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
	public ChassisSpeeds speeds;

	@Override
	public void robotInit() {
		this.m_swerve.resetPose(FieldPositions.Base);

		Limelight.registerDevice("Limelight", Optional.empty());

		NamedCommands.registerCommand("RaiseElevatorL1", this.elevator.RaiseToL1);
		NamedCommands.registerCommand("RaiseElevatorL2", this.elevator.RaiseToL2);
		NamedCommands.registerCommand("RaiseElevatorL3", this.elevator.RaiseToL3);
		NamedCommands.registerCommand("RaiseElevatorL4", this.elevator.RaiseToL4);
		NamedCommands.registerCommand("RaiseElevatorStow", this.elevator.RaiseToStow);
		NamedCommands.registerCommand("Score", this.m_arm.ScoreAlgaeCommand);
		NamedCommands.registerCommand("Intake", this.m_arm.IntakeAlgaeCommand);
		NamedCommands.registerCommand("IntakeCoral", this.m_arm.IntakeCoralCommand);
		NamedCommands.registerCommand("ScoreCoral", this.m_arm.ScoreCoralCommand);
	}

	@Override
	public void robotPeriodic() {
		m_swerve.dashboardPrint();
		SmartDashboard.putNumber("ElevatorHeight", this.elevator.getHeight());

		if (m_controller.getRightBumperButtonPressed()) {
			this.m_swerve.resetPose(FieldPositions.Base);
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
		new PathPlannerAuto("Base-R3L-CSR")
				.beforeStarting(Commands.runOnce(() -> System.out.println("We are starting!")))
				.andThen(Commands.runOnce(() -> m_swerve.Stop()))
				.andThen(Commands.runOnce(() -> System.out.println("We are done!")))
				.schedule();

		// new PathPlannerAuto("Example Skew Auto").schedule();
	}

	@Override

	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopPeriodic() {
		driveWithJoystick(true);
	}

	private void driveWithJoystick(boolean fieldRelative) {
		double overclock = 3;
		// boolean overclocked;

		if (m_controller.getAButton()) {
			overclock = Drivetrain.kMaxSpeed;
		}

		if (m_controller.getPOV() == 45) {
			// this.elevator.setPositions(ElevatorPositions.L1);
		}

		if (m_controller.getPOV() == 0) {
			// this.elevator.setPositions(ElevatorPositions.Stowed);
			this.m_arm.setTargetRotation(Rotation2d.fromDegrees(0));
		}

		if (m_controller.getPOV() == 180) {
			// this.elevator.setPositions(ElevatorPositions.L2);
			this.m_arm.setTargetRotation(Rotation2d.fromDegrees(160));
		}

		if (m_controller.getPOV() == 90) {
			// this.elevator.setPositions(ElevatorPositions.L3);
			this.m_arm.setTargetRotation(Rotation2d.fromDegrees(90));
		}

		if (m_controller.getXButton()) {
			//algae intake
			m_arm.intakeAlgae();
		} else if (m_controller.getBButton()) {
			m_arm.scoreAlgae();
		} else {
			m_arm.stopAlgae();
		}

		if (m_controller.getAButton()) {
			//algae score
			m_arm.intakeCoral();
		} else if (m_controller.getYButton()) {
			m_arm.scoreCoral();
		} else {
			m_arm.stopCoral();
		}



		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed = xFilter.calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftY(), 0.10), 3))
								* (overclock);

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = yFilter.calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftX(), 0.1), 3))
				* overclock;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		// final var rot =
		// -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(),
		// 0.1)) * Drivetrain.kMaxAngularSpeed;

		double rot = Math.pow(MathUtil.applyDeadband(-m_controller.getRightX(), 0.10), 3);

		ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

		m_swerve.Drive(speeds, fieldRelative);
	}
}