package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class UnlockClimberCommand extends Command{
    private final ClimberSubsystem m_ClimberSubsystem;


    public UnlockClimberCommand(ClimberSubsystem subsystem){
        m_ClimberSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_ClimberSubsystem.unlock();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
