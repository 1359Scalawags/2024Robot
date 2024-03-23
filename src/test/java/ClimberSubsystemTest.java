import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;
import frc.robot.subsystems.ClimberSubsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ClimberSubsystemTest {
    static final double DELTA = 1e-2;
    ClimberSubsystem climber;
    SendableCANSparkMax leftClimbMotor;
    SendableCANSparkMax rightClimbMotor;
    
    @BeforeEach
    void setup(){
        assert HAL.initialize(500, 0);
        climber = new ClimberSubsystem();
    }

    @AfterEach
    void shutdown() throws Exception{
        climber.close();
    }

    @Test
    void movesWhenUnlocked() throws Exception{
        climber.unlock();
        climber.setSpeed(0.5D);
        //wait(1000);
        System.out.println(climber.getLeftSpeed());
        assertEquals(0.5D, climber.getLeftSpeed(), DELTA);
        //assert true;
    }

    @Test
    void dosentMoveWhenLocked(){
        climber.lock();
        climber.setSpeed(1.0D);
        assertEquals(0.0D, climber.getLeftSpeed(), DELTA);
    }

    
}
