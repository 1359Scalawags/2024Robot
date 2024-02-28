package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class LockClimberCommand extends Command{
    private final ClimberSubsystem m_ClimberSubsystem;


    public LockClimberCommand(ClimberSubsystem subsystem){
        m_ClimberSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_ClimberSubsystem.lock();
        m_ClimberSubsystem.stopArm();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}
