// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.channels.FileLock;
import java.util.Optional;

import javax.xml.xpath.XPath;

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
	private final XboxController m_driver = new XboxController(0);
	private final XboxController m_operator = new XboxController(1);
	//private final Controller m_controllers;
	private final Elevator elevator = new Elevator(40, 8, 9);
	private final Arm m_arm = new Arm(50, 52, 51);

	private SendableChooser<Command> autoChooser;

	private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.3, Robot.kDefaultPeriod);
	private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.3, Robot.kDefaultPeriod);
	public ChassisSpeeds speeds;

	@Override
	public void robotInit() {
		Limelight.registerDevice("limelight");

		this.m_swerve.resetPose(FieldPositions.Base);

		NamedCommands.registerCommand("RaiseElevatorL1", this.elevator.RaiseToL1);
		NamedCommands.registerCommand("RaiseElevatorL2", this.elevator.RaiseToL2);
		NamedCommands.registerCommand("RaiseElevatorL3", this.elevator.RaiseToL3);
		NamedCommands.registerCommand("RaiseElevatorL4", this.elevator.RaiseToL4);
		NamedCommands.registerCommand("RaiseElevatorStow", this.elevator.RaiseToStow);
		NamedCommands.registerCommand("ScoreAlgae", this.m_arm.ScoreAlgaeCommand);
		NamedCommands.registerCommand("IntakeAlgae", this.m_arm.IntakeAlgaeCommand);
		NamedCommands.registerCommand("IntakeCoral", this.m_arm.IntakeCoralCommand);
		NamedCommands.registerCommand("ScoreCoral", this.m_arm.ScoreCoralCommand);
		NamedCommands.registerCommand("TurnToNet", this.m_arm.turnToNet);
		NamedCommands.registerCommand("TurnToL4", this.m_arm.turnToL4);
		NamedCommands.registerCommand("TurnToZero", this.m_arm.TurnTo0);


		this.autoChooser = AutoBuilder.buildAutoChooser();
		// this.autoChooser.setDefaultOption("None", Commands.none());
		this.autoChooser.setDefaultOption("TestingCommands", new PathPlannerAuto("TestingCommands"));
		// this.autoChooser.addOption("Testing", );

		SmartDashboard.putData("Auto Chooser", this.autoChooser);
	}

	@Override

	public void robotPeriodic() {
		SmartDashboard.putNumber("ElevatorHeight", this.elevator.getHeight());

		// if (m_driver.getRawButtonPressed(7))
		// {
		// 	this.m_swerve.resetGyro(Rotation2d.fromDegrees(180));
		// }
		// if (m_driver.getStartButtonPressed()) {
		// 	// this.m_swerve.resetPose(FieldPositions.Base);
		// 	if (this.m_swerve.resetPoseWithLimelight())
		// 	{
		// 		// Rumble the controller if pose was reset to limelight estimate
		// 		Commands
		// 			.runEnd(() -> m_driver.setRumble(RumbleType.kBothRumble, 1), () -> m_driver.setRumble(RumbleType.kBothRumble, 0))
		// 			.withTimeout(0.2)
		// 			.schedule();
		// 	}
		// }
		if (m_driver.getRawButtonPressed(7))
		{
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

		autoChooser.getSelected().schedule();
	}

	@Override

	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void teleopPeriodic() {
		driveWithJoystick(true);
		SmartDashboard.putBoolean("button", m_driver.getRawButton(7));
	}

	private double curveJoystick(double joystickInput) 
	{
		return Math.copySign(Math.pow(MathUtil.applyDeadband(joystickInput, 0.2), 2), joystickInput);
	}

	private void driveWithJoystick(boolean fieldRelative) {
		double overclock = 2;
		// boolean overclocked;
		
		if (m_operator.getRightTriggerAxis() >= 0.5) {
			switch (m_operator.getPOV()) {
				case 0:
					elevator.setPositions(ElevatorPosition.L4);
				break;

				case 90:
					elevator.setPositions(ElevatorPosition.L3);
				break;
					
				case 180: 
					elevator.setPositions(ElevatorPosition.Algae);
				break;

				case 270:
					elevator.setPositions(ElevatorPosition.L2);
				break;
			}
		} else {
			switch (m_operator.getPOV()) {
				case 0:
					m_arm.turnToNet();				
				break;

				case 90:
					m_arm.turnToL4();
				break;

				case 180: 
					m_arm.turnTo0();
				break;

				case 270:
					m_arm.turnTo20();
				break;
			}
		}

		
		if (m_operator.getYButton()) {
			m_arm.intakeCoral();
			//m_arm.scoreCoral();
		} else if (m_operator.getAButton()){
			m_arm.scoreCoral();
			//m_arm.stopAlgae();
		} else {
			m_arm.stopCoral();
		}

		if (m_operator.getXButton()) {
			m_arm.intakeAlgae();
		} else if (m_operator.getBButton()) {
			m_arm.scoreAlgae();
		} else {
			m_arm.stopAlgae();
		}

		if (m_driver.getRightBumperButton()) {
			overclock = 3;
		}

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		var xSpeed = xFilter.calculate(curveJoystick(m_driver.getLeftY())) * overclock;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = yFilter.calculate(curveJoystick(m_driver.getLeftX())) * overclock;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		// final var rot =
		// -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driver.getRightX(),
		// 0.1)) * Drivetrain.kMaxAngularSpeed;

		if (m_driver.getPOV() == 0) {
			// this.elevator.setPositions(ElevatorPosition.L3);
			// this.m_arm.setTargetRotation(Rotation2d.fromDegrees(90));
			xSpeed = 2;
		}

		if(m_driver.getPOV() == 180 ) {
			xSpeed = -2;
		}

		double rot = curveJoystick(-m_driver.getRightX()) * overclock;

		ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

		m_swerve.Drive(speeds, fieldRelative);
	}
}
