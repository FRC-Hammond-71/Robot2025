// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//import java.io.Console;
//import java.lang.ModuleLayer.Controller;

//import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends TimedRobot {
	private final XboxController m_controller = new XboxController(0);
	private final Drivetrain m_swerve = new Drivetrain();
	private final Elevator elevator = new Elevator(40,8,9);
	private final Arm m_arm = new Arm(0, 1);

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(4);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

	private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
	private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.3, 0.02);

	@Override
	public void robotInit() {
		this.m_swerve.resetPose(FieldPositions.Base);
		
		Limelight.registerDevice("Main", Optional.empty());

		NamedCommands.registerCommand("RaiseElevatorL1",this.elevator.RaiseToL1);
		NamedCommands.registerCommand("RaiseElevatorL2",this.elevator.RaiseToL2);
		NamedCommands.registerCommand("RaiseElevatorL3",this.elevator.RaiseToL3);
		NamedCommands.registerCommand("RaiseElevatorL4",this.elevator.RaiseToL4);
		NamedCommands.registerCommand("RaiseElevatorStow",this.elevator.RaiseToStow);
	}

	@Override
	public void robotPeriodic() {
		m_swerve.dashboardPrint();
		SmartDashboard.putNumber("ElevatorHeight", this.elevator.getHeight());
		CommandScheduler.getInstance().run();
		if (m_controller.getYButtonPressed()) {
			this.m_swerve.resetPose(FieldPositions.Base);
			this.elevator.resetEncoder();
		}
		
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
		// This ensures the robot does not continue to move again after re-enabling!
		// Clears the drive rate limiter and etc.
		this.m_swerve.Stop();
		this.elevator.stop();
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
		double overclock = 2;
		// boolean overclocked;

		if (m_controller.getAButton()) {
			overclock = Drivetrain.kMaxSpeed;
		}

	//	if (m_controller.getBButton()) {
	//		this.elevator.setPositions(ElevatorPositions.L1);
	//	}
//
	//	if (m_controller.getXButton()) {
	//		this.elevator.setPositions(ElevatorPositions.Stowed);
	//	}
//
	//	if (m_controller.getLeftBumperButton()) {
	//		this.elevator.setPositions(ElevatorPositions.L2);
	//	}
//
	//	if (m_controller.getRightBumperButton()) {
	//		this.elevator.setPositions(ElevatorPositions.L3);
	//	}

	if (m_controller.getXButton()) {
		//algae intake
		m_arm.Intake();
	} else if (!m_controller.getXButton()) {
		m_arm.Stop();
	}

	if(m_controller.getBButton()) {
		//algae score
		m_arm.Score();
	} else if (!m_controller.getXButton()) {
		m_arm.Stop();
	}

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed = (m_controller.getPOV() == 0) ? xFilter.calculate(1) * overclock
				: (m_controller.getPOV() == 180) ? xFilter.calculate(-1) * overclock
						: xFilter.calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftY(), 0.10), 3))
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
