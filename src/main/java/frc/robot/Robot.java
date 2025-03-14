// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Attractors.Attractor;
import frc.robot.Attractors.LineAttractor;
import frc.robot.Attractors.PointAttractor;
import frc.robot.Attractors.Controllers.FaceController;
import frc.robot.Commands.CommandUtils;
import frc.robot.Commands.ControllerCommands;
import frc.robot.Commands.GameCommands;
import frc.robot.Limelight.Limelight;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorPosition;
import frc.robot.Utilities.ChassisSpeedsUtils;

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

	
	private Command semiAutomatedCommand = null;
	private FaceController[] coralStationControllers;
	private FaceController netController;

	@Override
	public void robotInit() 
	{
		System.out.println("Waiting for connection to Driver Station...");		
		while (!DriverStation.waitForDsConnection(2))
		{
			System.out.println("Retrying connection to Driver Station...");
		}
		System.out.println("Connected to Driver Station!");

		final var generatedReefWaypoints = ReefWaypointGenerator.generateHexagonPoses();
		ReefWaypointGenerator.printWaypoints(generatedReefWaypoints);
		{
			Pose2d coralStationLeft = FieldConstants.CoralStationLeftLineup();
			Pose2d coralStationRight = FieldConstants.CoralStationRightLineup();
			this.coralStationControllers = new FaceController[] {
				new FaceController(coralStationLeft.getRotation(), new PointAttractor(coralStationLeft.getTranslation(), 3)),
				new FaceController(coralStationRight.getRotation(), new PointAttractor(coralStationRight.getTranslation(), 3))
			};
		}

		this.netController = new FaceController(
			FieldConstants.AlgaeNetRotation(), 
			new LineAttractor(FieldConstants.AlgaeNetLine(), 1f));

		Limelight.registerDevice("limelight");

		initialPoseChooser.addOption("BR", FieldConstants.BR());
		initialPoseChooser.addOption("BM", FieldConstants.BM());

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
		NamedCommands.registerCommand("ScoreCoralL4", this.gCommands.ScoreCoraL4());
		NamedCommands.registerCommand("StowAll", this.gCommands.StowAll());

		this.autoChooser = AutoBuilder.buildAutoChooser();

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
		this.ElevatorLigament.setLength(0.762 + this.elevator.getHeight() / 39.37);
		this.ArmLigament.setAngle(Rotation2d.fromDegrees(90-82+180).minus(this.arm.getRotation()));

		if (driver.getStartButtonPressed()) {
			if (this.swerve.resetPoseWithLimelight())
			{
				ControllerCommands.Rumble(driver, 0.2).schedule();
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
	public void teleopPeriodic() 
	{
		driveWithJoystick(true);
	}

	@Override
	public void teleopInit() 
	{
		this.arm.setDefaultCommand(Commands.run(() -> {

			if (this.driver.getYButton())
			{
				this.arm.intakeAlgae();
				this.arm.intakeCoral();
			}
			else
			{
				this.arm.stopAlgae();
				this.arm.stopCoral();
			}

		}, this.arm));	
	}

	@Override
	public void teleopExit() 
	{
		this.arm.setDefaultCommand(null);
	}

	private void driveWithJoystick(boolean fieldRelative) {
		double overclock = 2;
		boolean overclocked;
		// 
		//  if (operator.getRightTriggerAxis() >= 0.5) {
		//  	switch (operator.getPOV()) {
		//  		case 0:
		//  			elevator.setPositions(Elevator.kL4Height);
		//  		break;

		//  		case 90:
		//  			elevator.setPositions(Elevator.kL3Height);
		//  		break;
		// 			// 
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
		//  			arm.turnToStowed();
		//  		break;

		//  		case 270:
		//  			arm.turnTo20();
		//  		break;
		//  	}
		//  }
		
		// if (this.driver.getYButtonPressed())
		// {
		// 	// this.arm.setTargetRotation(Rotation2d.fromDegrees(180));
		// 	this.gCommands
		// 		.ScoreCoral4()
		// 		.andThen(ControllerCommands.Rumble(driver, 0.2))
		// 		.finallyDo(() -> 
		// 		{
		// 			this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
		// 			this.elevator.setPositions(0);
		// 		})
		// 		.onlyWhile(this.driver::getYButton)
		// 		.schedule();
		// }
		// if (this.driver.getXButtonPressed())
		// {
		// 	this.arm.PivotTo180()
        //     	.andThen(Commands.deadline(this.arm.IntakeAlgaeCommand(), this.arm.PivotToHigherAlgae()))
		// 		.onlyWhile(this.driver::getXButton)
        //     	.andThen(this.arm.PivotTo180())
		// 		.finallyDo(() -> 
		// 		{
		// 			this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
		// 			this.elevator.setPositions(0);
		// 		})
		// 		.schedule();
		// }
		if (this.driver.getXButtonPressed())
		{
			this.gCommands.ScoreCoralL3()
				.onlyWhile(this.driver::getXButton)
				.finallyDo(() -> 
				{
					this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
					this.elevator.setPositions(0);
				})
				.schedule();
		}
		if (this.driver.getBButtonPressed())
		{
			this.gCommands.ScoreCoraL4()
				.onlyWhile(this.driver::getBButton)
				.finallyDo(() -> 
				{
					this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
					this.elevator.setPositions(0);
				})
				.schedule();
		}
		if (this.driver.getAButtonPressed())
		{
			this.gCommands
				.IntakeLowerAlgae()
				.andThen(ControllerCommands.Rumble(driver, 0.2))
				.finallyDo(() -> 
				{
					this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
					this.elevator.setPositions(0);
				})
				.onlyWhile(this.driver::getAButton)
				.schedule();
		}
		// if (this.driver.getAButton())
		// {
		// 	this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
		// 	this.elevator.setPositions(0);

		// 	// this.gCommands.IntakeAlgae().onlyWhile(this.driver::getAButton).schedule();
		// }
		
		// if (operator.getYButton()) {
		// 	arm.intakeCoral();
		// 	arm.scoreCoral();
		// } else if (operator.getAButton()){
		// 	arm.scoreCoral();
		// 	arm.stopAlgae();
		// } else {
		// 	arm.stopCoral();
		// }

		// if (operator.getXButton()) {
		// 	arm.intakeAlgae();
		// } else if (operator.getBButton()) {
		// 	arm.scoreAlgae();
		// } else {
		// 	arm.stopAlgae();
		//  }

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

		double rot = Math.pow(MathUtil.applyDeadband(-driver.getRightX(), 0.05), 3) * Math.PI * 0.90;

		ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

		ChassisSpeeds assistedSpeeds = computeAssistedSpeeds();
		speeds.vxMetersPerSecond += assistedSpeeds.vxMetersPerSecond;
		speeds.vyMetersPerSecond += assistedSpeeds.vyMetersPerSecond;
		speeds.omegaRadiansPerSecond += assistedSpeeds.omegaRadiansPerSecond;

		this.swerve.Drive(speeds, fieldRelative);
	}

	private ChassisSpeeds computeAssistedSpeeds()
	{
		ChassisSpeeds wantedMovement = new ChassisSpeeds();
		Command wantedCommand = null;

		final boolean useDriverAssisted = true;
		if (useDriverAssisted && driver.getLeftBumperButton())
		{
			int reefCoralLevelNumb = (int)SmartDashboard.getNumber("DriverAssisted/ReefCoralLevel", -1); // -1 and 0 is ignore and L1-L4
			int reefCoralSideNumb = (int)SmartDashboard.getNumber("DriverAssisted/ReefCoralSide", -1); // -1 is ignore 0 is left and 1 is right
			int reefAlgaeHighOrLowNumb = (int)SmartDashboard.getNumber("DriverAssisted/ReefAlgae", -1); // -1 is none 0 is low 1 is high
			int reefFaceNumb = (int)SmartDashboard.getNumber("DriverAssisted/ReefFace", -1); // -1 is none (ignore) and follow standard 0-5 naming

			boolean doAssistCoral = reefCoralLevelNumb > 0 && reefCoralSideNumb >= 0 && reefFaceNumb >= 0;
			boolean doAssistAlgaeReef = reefAlgaeHighOrLowNumb >= 0 && reefFaceNumb >= 0;
			boolean doAssistCoralStation = SmartDashboard.getBoolean("DriverAssisted/CoralStation", true);	

			// TODO: FIX ISSUES WHERE CORAL INTAKE CONTROLLER IS USED BUT THEN NOT ATTRACKING PREVENTING OTHER MAGNETS TO BE 
			// Assist drivers when they are near a coral station by aligning them and running intake commands.
			FaceController nearestController = Attractor.getNearestAttractor(this.swerve.getPose().getTranslation(), this.coralStationControllers);
			if (doAssistCoralStation)
			{
				if (nearestController != null)
				{
					wantedMovement = nearestController.calculate(this.swerve.getPose());
					double distance = nearestController.getMagnitude(this.swerve.getPose().getTranslation());
					if(distance > 0.9 && nearestController.isAtRotation())
					{
						wantedCommand = Commands
							.parallel(this.elevator.RaiseToCSIntake(), this.arm.PivotToStowed())
							.andThen(this.arm.IntakeCoral())
							.onlyWhile(() -> this.driver.getLeftBumperButton())
							.andThen(this.arm.IntakeCoralCommand().withTimeout(0.2))
							.finallyDo(() -> {
								this.elevator.Stow();
								this.arm.turnToStowed();
								// SmartDashboard.putBoolean("DriverAssisted/CoralStation", false);
							});
					} 
				}
			}
			else
			{
				if (Attractor.isInRange(this.swerve.getPose().getTranslation(), this.netController))
				{
					wantedMovement = this.netController.calculate(this.swerve.getPose());

					double distance = this.netController.getMagnitude(this.swerve.getPose().getTranslation());
					if(distance > 0.9 && this.netController.isAtRotation())
					{
						wantedCommand = this.gCommands
							.ScoreNet()
							.onlyWhile(() -> this.driver.getLeftBumperButton())
							.finallyDo(() -> {
								this.elevator.Stow();
								this.arm.turnToStowed();
							});
					} 
				}
			}
		}

		if (wantedCommand != null && this.semiAutomatedCommand == null)
		{
			System.out.println("RAHHH");
			this.semiAutomatedCommand = wantedCommand;
			this.semiAutomatedCommand.finallyDo(() -> this.semiAutomatedCommand = null).schedule();
		}
		return wantedMovement;
	}

	private double curveJoystick(double joystickInput) 
	{
		return Math.copySign(Math.pow(MathUtil.applyDeadband(joystickInput, 0.05), 2), joystickInput);
	}
}