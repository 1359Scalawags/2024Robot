// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands.TimedCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeNoteInTimedShoot extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;
  private Timer startTimer;
  /**
   *command to turn belt on
   * 
   * @param subsystem The subsystem used by this command.
   */
  public IntakeNoteInTimedShoot(IntakeSubsystem subsystem) {
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
      if (startTimer.get() >= Constants.intakeSubsystem.kIntakeNoteInTime) {
        m_IntakeSubsystem.injectNote();      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopNoteMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    if (startTimer.get() >= Constants.intakeSubsystem.kStopIntakeInTime ) {
      return true;
    } else {
      return false;
    }   
  }
}
