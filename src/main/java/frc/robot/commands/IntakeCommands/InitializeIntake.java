package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class InitializeIntake extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private Timer startTimer;
    private boolean hasInitialized = false;

  /**
   *command to turn belt on
   * 
   * @param subsystem The subsystem used by this command.
   */
  public InitializeIntake(IntakeSubsystem subsystem) {
    m_IntakeSubsystem = subsystem;
    startTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        startTimer.reset();
        startTimer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (hasInitialized == false && startTimer.get() >= Constants.intakeSubsystem.kIntakeInitializeWaitTime) {
            m_IntakeSubsystem.initializeIntakeArm();
            hasInitialized = true;
        } 
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.enableIntakeArm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (startTimer.get() >= Constants.intakeSubsystem.kIntakeEnableWaitTime) {
            return true;
        } else {
            return false;
        }
    }
}
