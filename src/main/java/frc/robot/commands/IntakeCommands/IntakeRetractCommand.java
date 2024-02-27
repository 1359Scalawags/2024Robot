// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeRetractCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_IntakeSubsystem;
  private Timer noteInTimer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeRetractCommand(IntakeSubsystem subsystem) {
    m_IntakeSubsystem = subsystem;
    noteInTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Constants.kDebug) System.out.println("-------------Start Intake Retract-------------  ");
    m_IntakeSubsystem.positionUp();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(noteInTimer.get() >= Constants.intakeSubsystem.kInjectNoteCorrectionStartTime)
      m_IntakeSubsystem.injectNote();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_IntakeSubsystem.stopNoteMotors();    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(noteInTimer.get() >= Constants.intakeSubsystem.kInjectNoteCorrectionStopTime) {
      return true;
    } else {
      return false;
    }
  }
}
