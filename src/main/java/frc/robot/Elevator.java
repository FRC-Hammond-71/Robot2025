package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase {

    public static final double kGearing = 20; 
    public static final double kDiameter = 1.1279;
    public static final double kCircumference = kDiameter * Math.PI;
    
    public SparkMax elevatorMotor;
    
    public AbsoluteEncoder m_Encoder;
    private final PIDController m_controller = new PIDController(1,0,0);

    public Elevator(int driveMotorDeviceId,int encoderDeviceId) {

        this.elevatorMotor = new SparkMax(encoderDeviceId,MotorType.kBrushless);
        this.m_Encoder = elevatorMotor.getAbsoluteEncoder();

        // Distance per encoder is kCircumference
        // Distance per motor rotation kCircumference / 
    }
    
    

    /**
     * {@code getHeight()}
     * @return The current height of the elevator, in inches.
     */
    public double getHeight() 
    {
        return m_Encoder.getPosition() * kCircumference;
    }
    
    @Override
    public void periodic() {
        
        m_controller.calculate(1);


        // this.elevatorMotor.setVoltage();


        super.periodic();
    }

}


