package frc.robot;

// Import relevant classes.
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// 0 is one driver, 1 is two.

public class UNUSED extends SubsystemBase
{
    
    public static int controllersetting = 0;

    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry   odometry;
    AHRS                  gyro; 
    SwerveModule[]        swerveModules;
    
    // Constructor
    public UNUSED() 
    {

        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
        );
        
        gyro = new AHRS(NavXComType.kMXP_SPI);

        
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition()},
            new Pose2d(0,0,new Rotation2d()) 
        );
            
    }
    
    // Simple drive function
    public void Drive()
    { 

        // switch (controllersetting) {
        //     case 0:
      
        //     // These are the controls In the event that there is one driver.
        //     // These are the controls for the lettered buttons.
      
        //         // Open intake X
        //   if (m_controller.getXButton()) {
        //     rot = Math.PI / 2;
        //   }
        //   //  Close intake B
        //   if (m_controller.getBButton()) {
        //   }
        //   //  Deep Climb Controller 1 Y
        //   if (m_controller.getYButton()) {
        //   }    // Deep Climb Controller 2 A
        //   if (m_controller.getAButton()) {
        //   }
      
        //   // These are the controls for the D-PAD.
      
        // //  Score L3 LEFT
        //     if (m_controller.povLeft()) {
        //       }
        // //  Score L2 RIGHT
        //  if (m_controller.povRight()) {
        //   }
        // //  Score L4
        //     if (m_controller.povUp()) {
        //       }
        // //  Close intake B
        //      if (m_controller.povDown()) {
        //   }

        // if(m_controller)
      
        //   break;
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));
        
        // Get the SwerveModuleStates for each module given the desired speeds.
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
        // Output order is Front-Left, Front-Right, Back-Left, Back-Right
        
        swerveModules[0].setDesiredState(swerveModuleStates[0]);
        swerveModules[1].setDesiredState(swerveModuleStates[1]);
        swerveModules[2].setDesiredState(swerveModuleStates[2]);
        swerveModules[3].setDesiredState(swerveModuleStates[3]);
    }
    
    // Fetch the current swerve module positions.
    public SwerveModulePosition[] getCurrentSwerveModulePositions()
    {
        return new SwerveModulePosition[] {
            swerveModules[0].getPosition(), // Front-Left
            swerveModules[1].getPosition(), // Front-Right
            swerveModules[2].getPosition(), // Back-Left
            swerveModules[3].getPosition()  // Back-Right
        };
    }
    
    @Override
    public void periodic()
    {
        // Update the odometry every run.
    


    }
    
}