package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Temperature;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.configs.CANrangeConfiguration;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX flyWheelMotor;
    private final CANrange shooterCANrange;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final NeutralOut neutralRequest = new NeutralOut();
    
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> deviceTemp;

    public ShooterIOReal(){
        flyWheelMotor = new TalonFX(ShooterConstants.Real.kShooterMotorID, ShooterConstants.Real.kCanbusName);
        shooterCANrange = new CANrange(ShooterConstants.Real.kRangeSensorID, ShooterConstants.Real.kCanbusName);

        TalonFXConfiguration flyWheelMotorConfig = new TalonFXConfiguration();
        CANrangeConfiguration shooterCANrangeConfig = new CANrangeConfiguration();
        
        shooterCANrange.getConfigurator().apply(shooterCANrangeConfig);

        //Settings
        flyWheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Flywheel closed-loop gains
        flyWheelMotorConfig.Slot0.kP = ShooterConstants.Real.kFlywheelKp;
        flyWheelMotorConfig.Slot0.kI = ShooterConstants.Real.kFlywheelKi;
        flyWheelMotorConfig.Slot0.kD = ShooterConstants.Real.kFlywheelKd;
        flyWheelMotorConfig.Slot0.kS = ShooterConstants.Real.kFlywheelKs;
        flyWheelMotorConfig.Slot0.kV = ShooterConstants.Real.kFlywheelKv;
        flyWheelMotorConfig.Slot0.kA = ShooterConstants.Real.kFlywheelKa;

        // Current limiting
        flyWheelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flyWheelMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Real.kSupplyLimit;
        flyWheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flyWheelMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.Real.kStatorLimit;

        flyWheelMotor.getConfigurator().apply(flyWheelMotorConfig);

        velocity = flyWheelMotor.getVelocity();
        appliedVolts = flyWheelMotor.getMotorVoltage();
        statorCurrent = flyWheelMotor.getStatorCurrent();
        deviceTemp = flyWheelMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, statorCurrent, deviceTemp);

        flyWheelMotor.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        BaseStatusSignal.refreshAll(velocity, appliedVolts, statorCurrent, deviceTemp);

        inputs.flyWheelVelocityRotsPerSec = velocity.getValueAsDouble();
        inputs.flyWheelVoltage = appliedVolts.getValueAsDouble();
        inputs.flyWheelCurrent = statorCurrent.getValueAsDouble();
        inputs.flyWheelTemp = deviceTemp.getValueAsDouble();
        inputs.flyWheelReved = Math.abs(velocity.getValueAsDouble()) > 0.5;//flywheel thresh hold
        inputs.gamePieceDetected = shooterCANrange.getIsDetected().getValue();
    }

    @Override
    public void shoot() {
        revFlyWheel(ShooterConstants.Real.kFlywheelVelocitySetpoint);
    }
        
    public void revFlyWheel(double flyWheelVelocityRotsPerSec){
        flyWheelMotor.setControl(velocityRequest.withVelocity(flyWheelVelocityRotsPerSec));
    }

    public void setVoltage(double volts){
        flyWheelMotor.setControl(voltageRequest.withOutput(volts));
    }

    public void stop(){
        flyWheelMotor.setControl(voltageRequest.withOutput(0.0));
    }

    public void disableShooter(){
        flyWheelMotor.setControl(neutralRequest);
    }

}
