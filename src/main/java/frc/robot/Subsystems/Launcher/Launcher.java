package frc.robot.Subsystems.Launcher;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.CommandUtils;

public class Launcher extends SubsystemBase 
{
    private SparkMax algaeMotor;
    private SparkMax coralMotor;
    private boolean isAlgaeDetected = false;

    public Launcher(int algaeMotorDeviceID, int coralMotorDeviceID)
    {
        this.coralMotor = new SparkMax(coralMotorDeviceID, MotorType.kBrushless);

        this.algaeMotor = new SparkMax(algaeMotorDeviceID, MotorType.kBrushless);
        this.algaeMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command cmdScoreAlgae() { return CommandUtils.withName("ScoreAlgae", Commands.runEnd(() -> this.scoreAlgae(), () -> this.stopAlgae(), this).withTimeout(1)); } 
    public Command cmdAutoIntakeAlgae() { return CommandUtils.withName("IntakeAlgae", Commands.runEnd(() -> this.intakeAlgae(), () -> this.stopAlgae(), this).onlyWhile(() -> !isAlgaeDetected).andThen(Commands.runEnd(() -> this.intakeAlgae(), () -> this.stopAlgae(), this).withTimeout(0.5))); }
    public Command cmdIntakeAlgae() { return CommandUtils.withName("IntakeAlgae", Commands.runEnd(() -> this.intakeAlgae(), () -> this.stopAlgae(), this)); }
    public Command cmdIntakeCoral() { return CommandUtils.withName("IntakeCoral", Commands.runEnd(() -> this.intakeCoral(), () -> this.stopCoral(), this)); }
    public Command cmdScoreCoral() { return CommandUtils.withName("ScoreCoral", Commands.runEnd(() -> this.scoreCoral(), () -> this.stopCoral(), this)); }

        @Override
    public void periodic() {

        if (!this.isAlgaeDetected && this.algaeMotor.getOutputCurrent() > 75 && algaeMotor.get() > 0)
        {
            this.isAlgaeDetected = true;
        }

        if (this.isAlgaeDetected && this.algaeMotor.getOutputCurrent() < 15 && this.algaeMotor.getOutputCurrent() > 1 && algaeMotor.get() < 0)
        {
            this.isAlgaeDetected = false;
        }

        SmartDashboard.putBoolean("Launcher/IsAlgaeDetected", this.isAlgaeDetected);
        SmartDashboard.putNumber("Launcher/AlgaeMotorCurrent", this.algaeMotor.getOutputCurrent());
    }

    public void stop()
    {
        this.stopAlgae();
        this.stopCoral();
    }

    public void intakeCoral() {
        this.coralMotor.set(-0.6);
    }
    
    public void scoreCoral() 
    {
        this.coralMotor.set(0.2);
    }

    public void intakeAlgae() {
        this.algaeMotor.set(1);
    }

    public void scoreAlgae() {
        this.algaeMotor.set(-1);
    }

    public void stopAlgae() 
    {
        this.algaeMotor.stopMotor();
    }

    public void stopCoral() {
        this.coralMotor.stopMotor();
    }

}
