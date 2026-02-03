package frc.robot.subsystems;




import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BufferSubsystem extends SubsystemBase {
private SparkMax motorVortex;

  public BufferSubsystem() {
    this.motorVortex = new SparkMax(12, MotorType.kBrushless);
    
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kCoast)
        .inverted(true);

    // Persist parameters to retain configuration in the event of a power cycle
    motorVortex.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // this.motorVortex.follow(this.motorLeftFront);

    // this.drive = new DifferentialDrive(this.motorRightFront, this.motorLeftFront);

  }

    public void percentOut(double speed) {
      motorVortex.set(speed);
    }

    public void stop() {
      motorVortex.stopMotor();
    }


}