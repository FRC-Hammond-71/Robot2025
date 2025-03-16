package frc.robot.Subsystems.Arm;

import org.dyn4j.geometry.Rotation;

import com.pathplanner.lib.auto.CommandUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ColorSensorV3;
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
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Commands.CommandUtils;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase {

    public static final double kGearing = (17 / 3) * (15 / 4) * (52 / 14);
    public static final Rotation2d kMaxRotation = Rotation2d.fromDegrees(220);
    public static final Rotation2d kMinRotation = Rotation2d.fromDegrees(0);

    private Rotation2d targetRotation;
    private ProfiledPIDController PID = new ProfiledPIDController(3, 0, 0.0005, new Constraints(Math.PI * 2, Math.PI * 0.9));
    private ArmFeedforward feedforward;
    private SparkMax rotationMotor;
    private AbsoluteEncoder absoluteEncoder;

    public static final Rotation2d kNetAngle = Rotation2d.fromDegrees(125);
    public static final Rotation2d k180Angle = Rotation2d.fromDegrees(180);
    public static final Rotation2d kL4Angle = Rotation2d.fromDegrees(180);
    public static final Rotation2d kLowerAlgaeAngle = Rotation2d.fromDegrees(20);
    public static final Rotation2d kHigherAlgaeAngle = Rotation2d.fromDegrees(210);
    public static final Rotation2d kStowedAngle = Rotation2d.fromDegrees(0);

    public Command PivotToNet() { return CommandUtils.withName("PivotToNet", makePivotCommand(kNetAngle)); }
    public Command PivotTo180() { return CommandUtils.withName("PivotTo180", makePivotCommand(k180Angle)); }
    public Command PivotToL4Coral() { return CommandUtils.withName("PivotToL4Coral", makePivotCommand(kL4Angle)); }
    public Command PivotToLowerAlgae() { return CommandUtils.withName("PivotToLowerAlgae", makePivotCommand(kLowerAlgaeAngle)); }
    public Command PivotToHigherAlgae() { return CommandUtils.withName("PivotToHigherAlgae", makePivotCommand(kHigherAlgaeAngle)); }
    public Command PivotToStowed() { return CommandUtils.withName("PivotToStowed", makePivotCommand(kStowedAngle)); }

    public Arm(int rotationMotorDeviceID) {
            
        this.PID.setTolerance(Math.toRadians(2));

        this.rotationMotor = new SparkMax(rotationMotorDeviceID, MotorType.kBrushless);
        this.absoluteEncoder = rotationMotor.getAbsoluteEncoder();

        this.targetRotation = new Rotation2d();

        this.feedforward = new ArmFeedforward(0.05, 1.2, 1.5, 0);

        this.rotationMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command makePivotCommand(Rotation2d rot)
    {
        return Commands.run(() -> this.setTargetRotation(rot), this).until(() -> this.isAtTarget());
    }

    public void stop()
    {
        this.rotationMotor.stopMotor();
        this.PID.reset(this.getRotation().getRadians());
        this.setTargetRotation(this.getRotation());
    }

    public boolean isAtTarget() {
       return this.PID.atGoal();
    }

    public void turnToNet() {  
        setTargetRotation(Rotation2d.fromDegrees(130));
    }

    public void turnToL4() {
        setTargetRotation(Rotation2d.fromDegrees(185));
    }

    public void turnToStowed() {
        setTargetRotation(Rotation2d.fromDegrees(0));
    }

    public void turnTo20() {
        setTargetRotation(Rotation2d.fromDegrees(35));
    } 

    public void setTargetRotation(Rotation2d target) {
        this.PID.setGoal(target.getRadians());
        this.targetRotation = target;
    }

    public Rotation2d getRotation() {
        double rot = this.absoluteEncoder.getPosition() * 360;
        rot = rot >= 0 ? rot : (360 + rot);
        rot += 1;

        if (rot > 350)
        {
            rot = 0;
        }

        return Rotation2d.fromDegrees(rot);
    }

    @Override
    public void periodic() 
    {
        Rotation2d currentRot = this.getRotation();

        // SmartDashboard.putNumber("Arm Target", this.targetRotation.getDegrees());
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
        SmartDashboard.putNumber("Arm Measured Speed", this.absoluteEncoder.getVelocity() * 360);

        final double voltage = feedforward.calculate(getRotation().getRadians()-Math.PI/2, PIDEffort);

        // SmartDashboard.putNumber("Arm Voltage", voltage);

        // TODO: Inverse arm rotation motor!
        this.rotationMotor.setVoltage(-voltage);
    }
}