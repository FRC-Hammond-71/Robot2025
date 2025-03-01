package frc.robot;

import org.dyn4j.geometry.Rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase {

    public static final double kGearing = (17 / 3) * (15 / 4) * (20 / 7);
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
        this.feedforward = new ArmFeedforward(0, 0, 3, 0);
        this.PID = new ProfiledPIDController(.1, 0, 0, new Constraints(30, 5));

    }

    public Command ScoreAlgaeCommand = Commands.run(() -> this.scoreAlgae(), this);
    public Command IntakeAlgaeCommand = Commands.run(() -> this.intakeAlgae(), this);
    public Command IntakeCoralCommand = Commands.run(() -> this.intakeCoral(), this);
    public Command ScoreCoralCommand = Commands.run(() -> this.scoreCoral(), this);

    public void intakeCoral() {
        this.coralMotor.set(-0.5);
    }

    public void scoreCoral() {
        this.coralMotor.set(0.5);
    }

    public void intakeAlgae() {
        this.algaeMotor.set(-0.5);
    }

    public void scoreAlgae() {
        this.algaeMotor.set(0.8);
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
        return Rotation2d.fromDegrees(this.absoluteEncoder.getPosition() * 360);
    }
    

    @Override
    public void periodic() 
    {
        // double PIDEffort = PID.calculate(this.getRotation().getDegrees(), this.targetRotation.getDegrees());

        // if (getRotation().getDegrees() >= kMaxRotation.getDegrees() && PIDEffort > 0) {

        //     this.rotationMotor.stopMotor();
        //     return;
        // }
        // if (getRotation().getDegrees() <= kMaxRotation.getDegrees() && PIDEffort < 0) {

        //     this.rotationMotor.stopMotor();
        //     return;
        // }
        // this.rotationMotor.setVoltage(feedforward.calculate(getRotation().getRadians()-Math.PI/2, PIDEffort));
    }
}