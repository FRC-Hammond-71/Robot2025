// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Arm.Arm;
import frc.robot.Commands.ControllerCommands;
import frc.robot.Commands.GameCommands;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Elevator.Elevator;
import frc.robot.Elevator.ElevatorPosition;
import frc.robot.Limelight.Limelight;

public class Robot extends TimedRobot {

	private final XboxController driver = new XboxController(0);
	private final XboxController operator = new XboxController(1);

	//#region Subsystems
	public final Drivetrain swerve = new Drivetrain();
	public final Elevator elevator = new Elevator(40, 8, 9);
	public final Arm arm = new Arm(50, 52, 51);
	//#endregion

	private SendableChooser<Command> autoChooser;
	private SendableChooser<Pose2d> initialPoseChooser = new SendableChooser<>();

	private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.6, Robot.kDefaultPeriod);
	private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.6, Robot.kDefaultPeriod);
	private final LinearFilter rFilter = LinearFilter.singlePoleIIR(0.6, Robot.kDefaultPeriod);
	public ChassisSpeeds speeds;

	public GameCommands gCommands;

	private Mechanism2d Mechanism;
	private MechanismLigament2d ElevatorLigament;
	private MechanismLigament2d ArmLigament;

	@Override
	public void robotInit() {

		Limelight.registerDevice("limelight");

		initialPoseChooser.addOption("BR", FieldPositions.BR);
		initialPoseChooser.addOption("BM", FieldPositions.BM);

		initialPoseChooser.onChange((chosenPose) -> {
			this.swerve.resetPose(chosenPose);
			System.out.printf("Reset pose to (%f, %f) at %f\n", chosenPose.getX(), chosenPose.getY(), chosenPose.getRotation().getDegrees());
		});

		SmartDashboard.putData("Initial Pose Chooser", this.initialPoseChooser);

		this.gCommands = new GameCommands(this);

		//#region PathPlanner Configuration
		NamedCommands.registerCommand("ScoreNet", this.gCommands.ScoreNet());
		NamedCommands.registerCommand("RaiseElevatorL1", this.elevator.RaiseToL1());
		NamedCommands.registerCommand("RaiseElevatorL2", this.elevator.RaiseToL2());
		NamedCommands.registerCommand("RaiseElevatorL3", this.elevator.RaiseToL3());
		NamedCommands.registerCommand("RaiseElevatorL4", this.elevator.RaiseToL4());
		NamedCommands.registerCommand("RaiseElevatorStow", this.elevator.RaiseToStowed());
		NamedCommands.registerCommand("PivotToStowed", this.arm.PivotToStowed());
		NamedCommands.registerCommand("ScoreAlgae", this.arm.ScoreAlgaeCommand());
		NamedCommands.registerCommand("IntakeAlgae", this.arm.IntakeAlgaeCommand());
		NamedCommands.registerCommand("IntakeCoral", this.arm.IntakeCoralCommand());
		NamedCommands.registerCommand("ScoreCoral", this.arm.ScoreCoralCommand());
		NamedCommands.registerCommand("TurnToNet", this.arm.PivotToNet());
		NamedCommands.registerCommand("IntakeHigherAlgae", this.gCommands.IntakeHigherAlgae());
		NamedCommands.registerCommand("IntakeLowerAlgae", this.gCommands.IntakeLowerAlgae());
		NamedCommands.registerCommand("IntakeFromCS", this.gCommands.IntakeFromCS());
		NamedCommands.registerCommand("ScoreCoralL3", this.gCommands.ScoreCoralL3());
		NamedCommands.registerCommand("ScoreCoralL4", this.gCommands.ScoreCoral4());
		NamedCommands.registerCommand("StowAll", this.gCommands.StowAll());

		this.autoChooser = AutoBuilder.buildAutoChooser();
		this.autoChooser.setDefaultOption("BR-R4LC4", new PathPlannerAuto("BR-R4LC4"));

		SmartDashboard.putData("Auto Chooser", this.autoChooser);
		//#endregion

		this.Mechanism = new Mechanism2d(2, 2);
		this.ArmLigament = new MechanismLigament2d("Arm", 0.6, 0);
		this.ArmLigament.setColor(new Color8Bit("#eb4034"));

		this.ElevatorLigament = new MechanismLigament2d("Elevator", 0.762, 82);
		
		this.ElevatorLigament.append(this.ArmLigament);
		this.Mechanism.getRoot("Root", 1, 0).append(this.ElevatorLigament);

		SmartDashboard.putData("Mech2D", this.Mechanism);
	}

	@Override

	public void robotPeriodic() 
	{
		// SmartDashboard.putNumber("Arm/CoralProximity", this.arm.coralProximity());
		// SmartDashboard.putNumber("Elevator/ElevatorHeight", this.elevator.getHeight());
		// SmartDashboard.putBoolean("Arm/CoralIsDetected", this.arm.isCoralDetected());

		this.ElevatorLigament.setLength(0.762 + this.elevator.getHeight() / 39.37);
		this.ArmLigament.setAngle(Rotation2d.fromDegrees(90-82+180).minus(this.arm.getRotation()));

		if (driver.getStartButtonPressed()) {
			if (this.swerve.resetPoseWithLimelight())
			{
				ControllerCommands.Rumble(driver, 0.2).schedule();;
			}
		}
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
		// This ensures the robot does not continue to move again after re-enabling!
		// Clears the drive rate limiter and etc.
		this.xFilter.reset();
		this.yFilter.reset();
		this.rFilter.reset();
		this.swerve.Stop();
		this.elevator.stop();
		this.arm.stop();
	}

	@Override
	public void autonomousPeriodic() {

		swerve.updateOdometry();

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
		SmartDashboard.putBoolean("button", driver.getRawButton(7));
	}

	@Override
	public void teleopInit() 
	{
		
	}

	private double curveJoystick(double joystickInput) 
	{
		return Math.copySign(Math.pow(MathUtil.applyDeadband(joystickInput, 0.05), 2), joystickInput);
	}

	public void Lolipop() {
		elevator.setPositions(ElevatorPosition.Algae);
		this.arm.turnTo0();
	}
	
	public void LowerAlgae() {
		elevator.setPositions(ElevatorPosition.LowerAlgae);
		this.arm.turnTo20();
	}
	
	public void HigherAlgae() {
		this.arm.turnToL4();
		elevator.setPositions(ElevatorPosition.Stowed);
	}
	
	public void Net() {
		elevator.setPositions(ElevatorPosition.Max);
		this.arm.turnToNet();
	}

	private void driveWithJoystick(boolean fieldRelative) {
		double overclock = 2;
		// boolean overclocked;
		
		//  if (operator.getRightTriggerAxis() >= 0.5) {
		//  	switch (operator.getPOV()) {
		//  		case 0:
		//  			elevator.setPositions(Elevator.kL4Height);
		//  		break;

		//  		case 90:
		//  			elevator.setPositions(Elevator.kL3Height);
		//  		break;
					
		//  		case 180: 
		//  			elevator.setPositions(Elevator.kLowerAlgaeHeight);
		//  		break;

		//  		case 270:
		//  			elevator.setPositions(Elevator.kL2Height);
		//  		break;
		//  	}
		//  } else {
		//  	switch (operator.getPOV()) {
		//  		case 0:
		//  			arm.turnToNet();	
		//  		break;

		//  		case 90:
		//  			arm.turnToL4();
		//  		break;

		//  		case 180: 
		//  			arm.turnTo0();
		//  		break;

		//  		case 270:
		//  			arm.turnTo20();
		//  		break;
		//  	}
		//  }

		
		if (this.driver.getYButtonPressed())
		{
			// this.arm.setTargetRotation(Rotation2d.fromDegrees(180));
			this.gCommands
				.ScoreCoral4()
				.andThen(ControllerCommands.Rumble(driver, 0.2))
				.finallyDo(() -> 
				{
					// this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
					// this.elevator.setPositions(0);
				})
				.onlyWhile(this.driver::getYButton)
				.schedule();
		}
		if (this.driver.getXButtonPressed())
		{
			this.gCommands
				.ScoreNet()
				.andThen(ControllerCommands.Rumble(driver, 0.2))
				.finallyDo(() -> 
				{
					this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
					this.elevator.setPositions(0);
				})
				.onlyWhile(this.driver::getXButton)
				.schedule();
		}
		if (this.driver.getAButton())
		{
			this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
			this.elevator.setPositions(0);

			// this.gCommands.IntakeAlgae().onlyWhile(this.driver::getAButton).schedule();
		}
		
		// if (operator.getYButton()) {
		// 	arm.intakeCoral();
		// 	//m_arm.scoreCoral();
		// } else if (operator.getAButton()){
		// 	arm.scoreCoral();
		// 	//m_arm.stopAlgae();
		// } else {
		// 	arm.stopCoral();
		// }

		// if (operator.getXButton()) {
		// 	arm.intakeAlgae();
		// } else if (operator.getBButton()) {
		// 	arm.scoreAlgae();
		// } else {
		// 	arm.stopAlgae();
		// }

		if (driver.getRightBumperButton()) {
			overclock = 3;
		}

		// TODO: INVERT CONTROLS AT COMP

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		var xSpeed = xFilter.calculate(curveJoystick(driver.getLeftY())) * overclock;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = yFilter.calculate(curveJoystick(driver.getLeftX())) * overclock;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		// final var rot =
		// -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driver.getRightX(),
		// 0.1)) * Drivetrain.kMaxAngularSpeed;

		// if (driver.getPOV() == 0) {
			// this.elevator.setPositions(ElevatorPosition.L3);
			// this.m_arm.setTargetRotation(Rotation2d.fromDegrees(90));
			// xSpeed = 2;
		// }

		// if(driver.getPOV() == 180 ) {
			// xSpeed = -2;
		// }

		double rot = Math.pow(MathUtil.applyDeadband(-driver.getRightX(), 0.05), 3) * Math.PI;
		// double rot = curveJoystick(-driver.getRightX()) * overclock;

		ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

		swerve.Drive(speeds, fieldRelative);
	}
}