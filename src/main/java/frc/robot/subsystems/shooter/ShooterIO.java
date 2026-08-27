package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for shoot system with fly wheel and CANrange sensor.
 */

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        //Put the inputs I think should be used
        public double flyWheelVelocityRotsPerSec = 0.0;
        public double flyWheelVoltage = 0.0;
        public double flyWheelCurrent = 0.0;
        public double flyWheelTemp = 0.0;
        public boolean flyWheelReved = false;
        public boolean gamePieceDetected = false;}

    public default void updateInputs(ShooterIOInputs inputs){
    }
    
    public default void shoot(){
        //Ued to shoot balls
    }
    
    public default void revFlyWheel(double flyWheelVelocityRotsPerSec){
        //Used to run the flywheel and get to speed
    }

    public default void setVoltage(double volts){
        //Setting exact motor voltage
    }

    public default void stop(){
        //Setting motor velocity to 0.0 or stop
    }

    public default void disableShooter(){
        //Used to stop fly wheel like a break
    }
}