package frc.robot.Utilities;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtils 
{
    public static boolean isEmpty(ChassisSpeeds speeds)
    {
        return speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0;
    }    
}
