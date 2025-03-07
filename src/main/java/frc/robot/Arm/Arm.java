package frc.robot.Arm;

import org.dyn4j.geometry.Rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase {

    public static final double kGearing = (17 / 3) * (15 / 4) * (52 / 14);
    public static final Rotation2d kMaxRotation = Rotation2d.fromDegrees(180);
    public static final Rotation2d kMinRotation = Rotation2d.fromDegrees(0);

    private Rotation2d targetRotation;
    private ProfiledPIDController PID;
    private ArmFeedforward feedforward;
    private SparkMax rotationMotor;
    private SparkMax algaeMotor;
    private SparkMax coralMotor;
    private AbsoluteEncoder absoluteEncoder;

    public Arm(int rotationMotorDeviceID, int algaeMotorDeviceID, int coralMotorDeviceID) {

        this.rotationMotor = new SparkMax(rotationMotorDeviceID, MotorType.kBrushless);
        this.coralMotor = new SparkMax(coralMotorDeviceID, MotorType.kBrushless);
        this.algaeMotor = new SparkMax(algaeMotorDeviceID, MotorType.kBrushless);
        this.absoluteEncoder = rotationMotor.getAbsoluteEncoder();
        this.targetRotation = new Rotation2d(); // In degrees goofy :)
        this.feedforward = new ArmFeedforward(0, 0.8, 2, 0);
        this.PID = new ProfiledPIDController(1.5, 0, 0.0005, new Constraints(Math.PI / 2, Math.PI / 6));

        this.rotationMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command ScoreAlgaeCommand = Commands.run(() -> this.scoreAlgae(), this);
    public Command IntakeAlgaeCommand = Commands.run(() -> this.intakeAlgae(), this);
    public Command IntakeCoralCommand = Commands.run(() -> this.intakeCoral(), this);
    public Command ScoreCoralCommand = Commands.run(() -> this.scoreCoral(), this);

    public void stop()
    {
        stopAlgae();
        stopCoral();
        this.rotationMotor.stopMotor();
        this.PID.reset(this.getRotation().getRadians());
        this.PID.setGoal(this.getRotation().getRadians());
    }

    public void intakeCoral() {
        this.coralMotor.set(-0.8);
    }

    public void scoreCoral() {
        this.coralMotor.set(0.8);
    }


    public void intakeAlgae() {
        this.algaeMotor.set(-1);
    }

    public void scoreAlgae() {
        this.algaeMotor.set(1);
    }

    public void stopAlgae() {
        this.algaeMotor.stopMotor();
    }

    public void stopCoral() {
        this.coralMotor.stopMotor();
    }

    public void setTargetRotation(Rotation2d target) {
        this.targetRotation = target;
    }

    public Rotation2d getRotation() {
        double rot = this.absoluteEncoder.getPosition() * 360;
        rot = rot >= 0 ? rot : (360 + rot);
        rot += 1;

        if (rot > 358)
        {
            rot = 0;
        }

        return Rotation2d.fromDegrees(rot);
    }

    @Override
    public void periodic() {

        Rotation2d currentRot = this.getRotation();

        SmartDashboard.putNumber("Arm Target", this.targetRotation.getDegrees());
        SmartDashboard.putNumber("Arm Current", currentRot.getDegrees());
        // System.out.println(this.getRotation().getDegrees());

        // How much to move every 20ms essentially
        double PIDEffort = PID.calculate(currentRot.getRadians(), this.targetRotation.getRadians());

        if (currentRot.getDegrees() >= kMaxRotation.getDegrees() && PIDEffort > 0) 
        {
            this.rotationMotor.stopMotor();
            return;
        }

        if (currentRot.getDegrees() <= kMinRotation.getDegrees() && PIDEffort < 0)
        {
            this.rotationMotor.stopMotor();
            return;
        }

        SmartDashboard.putNumber("Arm Control Speed", PIDEffort * (1 / Robot.kDefaultPeriod));
        SmartDashboard.putNumber("Arm Measured Speed", this.absoluteEncoder.getVelocity() / 60 * 360);

        final double voltage = feedforward.calculate(getRotation().getRadians()-Math.PI/2, PIDEffort);

        SmartDashboard.putNumber("Arm Voltage", voltage);

        // TODO: Inverse arm rotation motor!
        this.rotationMotor.setVoltage(-voltage);
    }
}