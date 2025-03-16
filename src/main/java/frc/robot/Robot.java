// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Attractors.Attractor;
import frc.robot.Attractors.PointAttractor;
import frc.robot.Attractors.Controllers.FaceController;
import frc.robot.Attractors.Controllers.ReefPoseController;
import frc.robot.Commands.CommandUtils;
import frc.robot.Commands.ControllerCommands;
import frc.robot.Commands.GameCommands;
import frc.robot.DriverAssistance.DriverAssistOutput;
import frc.robot.Limelight.Limelight;
import frc.robot.Limelight.LimelightHelpers;
// import frc.robot.ReefWaypointGenerator.ReefEdgeWaypoints;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Drivetrain.Drivetrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Launcher.Launcher;
import frc.robot.Utilities.ChassisSpeedsUtils;
public class Robot extends TimedRobot 
{
	public final XboxController driverController = new XboxController(0);
	private final XboxController operatorController = new XboxController(1);

	//#region Subsystems
	public final Drivetrain swerve = new Drivetrain();
	public final Elevator elevator = new Elevator(40, 8, 9);
	public final Arm arm = new Arm(50);
	public final Launcher launcher = new Launcher(52, 51);
	//#endregion

	private SendableChooser<Command> autoChooser;
	private SendableChooser<Pose2d> initialPoseChooser = new SendableChooser<>();

	private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.6, Robot.kDefaultPeriod);
	private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.6, Robot.kDefaultPeriod);
	private final LinearFilter rFilter = LinearFilter.singlePoleIIR(0.6, Robot.kDefaultPeriod);
	public ChassisSpeeds speeds;

	public GameCommands gameCommands;
	public Limelight Limelight;

	private Mechanism2d Mechanism;
	private MechanismLigament2d ElevatorLigament;
	private MechanismLigament2d ArmLigament;
	
	private Command semiAutomatedCommand = null;
	private DriverAssistance driverAssistance;
	//private GameCommands gameCommands;

	@Override
	public void robotInit() 
	{
		System.out.println("Waiting for connection to Driver Station...");		
		while (!DriverStation.waitForDsConnection(2))
		{
			System.out.println("Retrying connection to Driver Station...");
		}
		System.out.println("Connected to Driver Station!");

		frc.robot.Limelight.Limelight.registerDevice("limelight");

		initialPoseChooser.addOption("BR", FieldConstants.BR());
		initialPoseChooser.setDefaultOption("BM", FieldConstants.BM());
		initialPoseChooser.addOption("BL", FieldConstants.BL());

		//baller 

		initialPoseChooser.onChange((chosenPose) -> {
			this.swerve.resetPose(chosenPose);
			System.out.printf("Reset pose to (%f, %f) at %f\n", chosenPose.getX(), chosenPose.getY(), chosenPose.getRotation().getDegrees());
		});

		SmartDashboard.putData("Initial Pose Chooser", this.initialPoseChooser);

		this.gameCommands = new GameCommands(this);

		//#region PathPlanner Configuration
		NamedCommands.registerCommand("ScoreNet", this.gameCommands.ScoreNet());
		NamedCommands.registerCommand("RaiseElevatorL1", this.elevator.RaiseToL1());
		NamedCommands.registerCommand("RaiseElevatorL2", this.elevator.RaiseToL2());
		NamedCommands.registerCommand("RaiseElevatorL3", this.elevator.RaiseToL3());
		NamedCommands.registerCommand("RaiseElevatorL4", this.elevator.RaiseToL4());
		NamedCommands.registerCommand("RaiseElevatorStow", this.elevator.RaiseToStowed());
		NamedCommands.registerCommand("PivotToStowed", this.arm.PivotToStowed());
		NamedCommands.registerCommand("ScoreAlgae", this.launcher.cmdScoreAlgae());
		NamedCommands.registerCommand("IntakeAlgae", this.launcher.cmdAutoIntakeAlgae());
		NamedCommands.registerCommand("IntakeCoral", this.launcher.cmdIntakeCoral());
		NamedCommands.registerCommand("ScoreCoral", this.launcher.cmdScoreCoral());
		NamedCommands.registerCommand("TurnToNet", this.arm.PivotToNet());
		NamedCommands.registerCommand("IntakeHigherAlgae", this.gameCommands.IntakeHigherAlgae());
		NamedCommands.registerCommand("IntakeLowerAlgae", this.gameCommands.IntakeLowerAlgae());
		NamedCommands.registerCommand("IntakeFromCS", this.gameCommands.IntakeFromCS());
		NamedCommands.registerCommand("ScoreCoralL3", this.gameCommands.ScoreCoralL3());
		NamedCommands.registerCommand("ScoreCoralL4", this.gameCommands.ScoreCoraL4());
		NamedCommands.registerCommand("StowAll", this.gameCommands.StowAll());

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

		this.driverAssistance = new DriverAssistance(this);
	}

	@Override

	public void robotPeriodic() 
	{
		this.ElevatorLigament.setLength(0.762 + this.elevator.getHeight() / 39.37);
		this.ArmLigament.setAngle(Rotation2d.fromDegrees(90-82+180).minus(this.arm.getRotation()));

		if (driverController.getStartButtonPressed()) 
		{
			// CommandScheduler.getInstance().cancel(this.semiAutomatedCommand);
			if (this.swerve.resetPoseWithLimelight())
			{
				ControllerCommands.Rumble(driverController, 0.2).schedule();
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
		this.launcher.setDefaultCommand(Commands.run(() -> {

			if (this.operatorController.getYButton() || this.driverController.getYButton())
			{
				this.launcher.intakeAlgae();
				this.launcher.intakeCoral();
			}
			else
			{
				this.launcher.stopAlgae();
				this.launcher.stopCoral();
			}

			if(this.operatorController.getAButton()|| this.driverController.getAButton())
			{
	
				this.launcher.scoreAlgae();
				this.launcher.scoreCoral();
			}

			if(this.operatorController.getXButtonPressed())
			{
				this.gameCommands.StowAll().schedule();
			}

			if(this.operatorController.getBButtonPressed())
			{
				this.gameCommands.RaiseToMax().schedule();
			}

			//D-PAD controls.
			if(this.operatorController.getPOV() == 0)
			{
				gameCommands.IntakeHigherAlgae().onlyWhile(() -> this.operatorController.getPOV() == 0).schedule();
			}
			if(this.operatorController.getPOV() == 90)
			{
				gameCommands.ScoreCoraL4().onlyWhile(() -> this.operatorController.getPOV() == 90).schedule();
			}
			if(this.operatorController.getPOV() == 180)
			{
				gameCommands.IntakeLowerAlgae().onlyWhile(() -> this.operatorController.getPOV() == 180).schedule();
			}
			if(this.operatorController.getPOV() == 270)
			{
				gameCommands.ScoreNet().onlyWhile(() -> this.operatorController.getPOV() == 270).schedule();
			}

		}, this.launcher));
	}

	private void driveWithJoystick(boolean fieldRelative) {
		double overclock = 3;

		if (this.driverController.getBButtonPressed())
		{
			this.gameCommands.ScoreProcessor().onlyWhile(this.driverController::getBButton);
		}
		
		// 	if (this.driverController.getAButton())
		// {
		// 	this.arm.setTargetRotation(Rotation2d.fromDegrees(0));
		// 	this.elevator.setPositions(0);
		// }

		if (driverController.getRightBumperButton()) 
		{
			overclock = 4.3;
		}

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.


		var xSpeed = (onRedAlliance()) 
			? -xFilter.calculate(curveJoystick(-driverController.getLeftY())) * overclock 
			:  xFilter.calculate(curveJoystick(-driverController.getLeftY())) * overclock;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed = (onRedAlliance()) 
			? -yFilter.calculate(curveJoystick(-driverController.getLeftX())) * overclock
			:  yFilter.calculate(curveJoystick(-driverController.getLeftX())) * overclock;

		double rot = Math.pow(MathUtil.applyDeadband(-driverController.getRightX(), 0.05), 3) * Math.PI * 0.90;

		ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

		ChassisSpeeds assistedSpeeds = assistDriver();
		if (this.driverController.getLeftBumperButton() && assistedSpeeds != null)
		{	
			if (Robot.isSimulation())
			{
				Pose2d pose = this.swerve.m_field.getRobotPose();
				pose = pose.plus(new Transform2d(
					new Translation2d(assistedSpeeds.vxMetersPerSecond, assistedSpeeds.vyMetersPerSecond * Robot.kDefaultPeriod),
					Rotation2d.fromDegrees(assistedSpeeds.omegaRadiansPerSecond * Robot.kDefaultPeriod)));
				this.swerve.m_field.setRobotPose(pose);
			}	
			else
			{
				speeds.vxMetersPerSecond += assistedSpeeds.vxMetersPerSecond;
				speeds.vyMetersPerSecond += assistedSpeeds.vyMetersPerSecond;
				speeds.omegaRadiansPerSecond += assistedSpeeds.omegaRadiansPerSecond;
			}
		}


		swerve.Drive(speeds, fieldRelative);
	}

	private ChassisSpeeds assistDriver()
	{
		var results = this.driverAssistance.assistDriver(this.driverController::getXButton);

		return ChassisSpeedsUtils.isEmpty(results.speeds) ? null : results.speeds;
	}

	private boolean onRedAlliance() {
		if (DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Red) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	}

	private double curveJoystick(double joystickInput) 
	{
		return Math.copySign(Math.pow(MathUtil.applyDeadband(joystickInput, 0.05), 2), joystickInput);
	}
}