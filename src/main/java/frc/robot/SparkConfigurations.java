package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

public class SparkConfigurations {
    
    public static SparkBaseConfig CoastMode = new SparkMaxConfig().idleMode(IdleMode.kCoast);
    public static SparkBaseConfig BreakMode = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    public static void ApplyConfigPersistNoReset(SparkMax motor, SparkBaseConfig config)
    {
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
