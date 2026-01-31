package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    private final TalonFX rightMotor;
    private final TalonFX leftMotor;
    private final Follower follower;


    private final DutyCycleOut percentOut = new DutyCycleOut(0);
    private final VelocityDutyCycle velocityOut = new VelocityDutyCycle(0);

      public ShooterSubsystem() {
        rightMotor = new TalonFX(10);
        leftMotor = new TalonFX(11);

        follower = new Follower(10, MotorAlignmentValue.Opposed);
         
        leftMotor.getConfigurator().apply(new TalonFXConfiguration());
        rightMotor.getConfigurator().apply(new TalonFXConfiguration());


        TalonFXConfigurator masterConfig = rightMotor.getConfigurator();
        TalonFXConfigurator slaveConfig = leftMotor.getConfigurator();

        
        
        // Slot0Configs positionPIDConfigs = new Slot0Configs();
        // positionPIDConfigs.kP = 0.2;
        // positionPIDConfigs.kI = 0.005;
        // positionPIDConfigs.kD = 0;
        // positionPIDConfigs.kA = 0;
        // masterConfig.apply(positionPIDConfigs);
        // slaveConfig.apply(positionPIDConfigs);
    
        MotorOutputConfigs rightMotoroutputConfigs = new MotorOutputConfigs();
        rightMotoroutputConfigs.PeakForwardDutyCycle = 1;
        rightMotoroutputConfigs.PeakReverseDutyCycle = -1;
        rightMotoroutputConfigs.withNeutralMode(NeutralModeValue.Coast);
        // rightMotoroutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        masterConfig.apply(rightMotoroutputConfigs);
        // slaveConfig.apply(rightMotoroutputConfigs);

        MotorOutputConfigs leftMotorOutputConfigs = new MotorOutputConfigs();
        leftMotorOutputConfigs.PeakForwardDutyCycle = 1;
        leftMotorOutputConfigs.PeakReverseDutyCycle = -1;
        leftMotorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        slaveConfig.apply(leftMotorOutputConfigs);

        SoftwareLimitSwitchConfigs limitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        limitSwitchConfigs.ForwardSoftLimitEnable = false;
        limitSwitchConfigs.ReverseSoftLimitEnable = false;
        limitSwitchConfigs.ForwardSoftLimitThreshold = 0;
        limitSwitchConfigs.ReverseSoftLimitThreshold = 0;
        masterConfig.apply(limitSwitchConfigs);
        slaveConfig.apply(limitSwitchConfigs);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = false;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 35;
        currentLimitsConfigs.SupplyCurrentLowerLimit = 30;
        currentLimitsConfigs.SupplyCurrentLowerTime = 1;
        masterConfig.apply(currentLimitsConfigs);
        slaveConfig.apply(currentLimitsConfigs);



        }



     public void percentOut(double speed) {
        rightMotor.setControl(percentOut.withOutput(speed));
        leftMotor.setControl(follower);
    }

    public void velocityOut(double speed) {
        rightMotor.setControl(velocityOut.withVelocity(speed));
        leftMotor.setControl(follower);
    }

    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public double getVelocity() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

     @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher Subsystem/Motors/Left/Voltage/Supply", leftMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Launcher Subsystem/Motors/Left/Current/Supply", leftMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Launcher Subsystem/Motors/Left/Current/Stator", leftMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Launcher Subsystem/Motors/Left/Velocity", leftMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Launcher Subsystem/Motors/Right/Voltage/Supply", rightMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Launcher Subsystem/Motors/Right/Current/Supply", rightMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Launcher Subsystem/Motors/Right/Current/Stator", rightMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Launcher Subsystem/Motors/Right/Velocity", rightMotor.getVelocity().getValueAsDouble());
    }
}
