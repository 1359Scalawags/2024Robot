package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/*
 * This command is responsible for faking encoder readings as live REV encoder values
 * are difficult to simulate. This command should NEVER run outside of simulation.
 */

public class SimValidRange extends Command {
    private final ClimberSubsystem m_ClimberSubsystem;

    public SimValidRange(ClimberSubsystem subsystem){
        m_ClimberSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if(RobotBase.isSimulation()){
            m_ClimberSubsystem.simulateEncoderInRange();
        }
        else{
            System.err.println("SIM COMMAND EXECUTING OUTSIDE OF SIMULATOR");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
