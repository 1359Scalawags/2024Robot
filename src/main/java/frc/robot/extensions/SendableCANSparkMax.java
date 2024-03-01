package frc.robot.extensions;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SendableCANSparkMax extends CANSparkMax implements Sendable {

    private RelativeEncoder encoder;

    public SendableCANSparkMax(int deviceId, MotorType motorType) {
        super(deviceId, motorType);
        String moduleName = "SparkMax";
        SendableRegistry.addLW(this, moduleName, deviceId);
        encoder = this.getEncoder();
    }

    public SendableCANSparkMax(int deviceId, MotorType motorType, SubsystemBase subsystem) {
        super(deviceId, motorType);
        String moduleName = "SparkMax";
        SendableRegistry.addLW(this, moduleName, deviceId);
        SendableRegistry.setSubsystem(this, subsystem.getName());
        encoder = this.getEncoder();       
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
        builder.addDoubleProperty("Position", this.encoder::getPosition, null);
        if(Constants.kDebug) {
            builder.addDoubleProperty("RPM?", this.encoder::getVelocity, null);
            builder.addBooleanProperty("Inverted?", this::getInverted, null);
            builder.addDoubleProperty("Temperature?", this::getMotorTemperature, null);
            builder.addStringProperty("IdleMode?", this::getIdleModeString, null);            
        }
    }

    public String getIdleModeString() {
        return this.getIdleMode().toString();
    }
    
}