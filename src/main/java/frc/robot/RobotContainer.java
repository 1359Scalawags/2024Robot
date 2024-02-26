// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmCommands.ExtendArmCommand;
import frc.robot.commands.ArmCommands.HomeClimberCommand;
import frc.robot.commands.ArmCommands.MoveClimberArms;
import frc.robot.commands.ArmCommands.RetractArmCommand;
import frc.robot.commands.IntakeCommands.IntakeWheelsOffCommand;
import frc.robot.commands.IntakeCommands.IntakeNoteInCommand;
import frc.robot.commands.IntakeCommands.IntakeNoteOutCommand;
import frc.robot.commands.IntakeCommands.HomeIntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeExtendCommand;
import frc.robot.commands.IntakeCommands.IntakeRetractCommand;
import frc.robot.commands.IntakeCommands.IntakeWheelsOffCommand;
import frc.robot.commands.IntakeCommands.IntakeNoteInCommand;
import frc.robot.commands.ShootingCommands.ShootCommand;
import frc.robot.commands.ShootingCommands.StopShootingCommand;
import frc.robot.commands.SwerveCommands.DriveForwardCommand;
import frc.robot.commands.SwerveCommands.FeildCentricDrive;
import frc.robot.commands.SwerveCommands.FieldCentricCommand;
import frc.robot.commands.SwerveCommands.ReverseDriveCommand;
import frc.robot.commands.SwerveCommands.RobotCentricCommand;
import frc.robot.commands.SwerveCommands.UnReverseDriveCommand;
import frc.robot.commands.SwerveCommands.ZeroGyroCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  

    
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "YAGSLConfigJSON/swerve/" + Constants.robotName));
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  //private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final Joystick driverJoystick = new Joystick(Constants.DriverJoystick.joystick);
  private final Joystick assistantJoystick = new Joystick(Constants.AssistantJoystick.joystick);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands(); 
  }
    /*
     *  SwerveSubsystem swerve,
     *  DoubleSupplier vX, 
     *  DoubleSupplier vY, 
     *  DoubleSupplier omega,
     *  DoubleSupplier throttle, 
     *  BooleanSupplier feildRelitive,
     *  boolean isOpenLoop
     */
  private void setDefaultCommands() {
    m_SwerveSubsystem.setDefaultCommand(
      new FeildCentricDrive(m_SwerveSubsystem,
      this::driverGetX,
      this::driverGetY,
      this::driverGetZ,
      this::driverGetThrottle,
      m_SwerveSubsystem::getFeildCentric, 
      false
      ));

    m_ClimberSubsystem.setDefaultCommand(
      new MoveClimberArms(m_ClimberSubsystem,
      this::assistantGetY
      ));
  }
  
  public double assistantGetY() {
    return -assistantJoystick.getY();
  }
  public double assistantGetX() {
    return -assistantJoystick.getX();
  }
  public double assistantGetZ() {
    return -assistantJoystick.getZ();
  }
  public double driverGetY() {
    return driverJoystick.getY();
  }

  public double driverGetX() {
    return -driverJoystick.getX();
  }

  public double driverGetZ() {
    return driverJoystick.getZ();
  }
  public double driverGetThrottle() {
    return driverJoystick.getThrottle();
  }
  


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //TODO: finilize button layout, communicate with drive team if possible. Not everything should be on driverJoystick.

    //Shooter commands/binds above
    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.shootButton)
      .onTrue(new ShootCommand(m_shooterSubsystem));
    
    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.shootButton)
      .onFalse(new StopShootingCommand(m_shooterSubsystem));


    //Climber commands/binds above
    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.extendClimberArmButton)
      .onTrue(new ExtendArmCommand(m_ClimberSubsystem));

    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.retractClimberArmButton)
      .onTrue(new RetractArmCommand(m_ClimberSubsystem));

    // intake commands/binds above
    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.intakeExtendButton)
      .onTrue(new IntakeExtendCommand(m_IntakeSubsystem));

    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.intakeRetractButton)
      .onTrue(new IntakeRetractCommand(m_IntakeSubsystem));

    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.intakeNoteInbutton)
      .onTrue(new IntakeNoteInCommand(m_IntakeSubsystem));

      new JoystickButton(assistantJoystick,Constants.AssistantJoystick.intakeNoteInbutton)
      .onFalse(new IntakeWheelsOffCommand(m_IntakeSubsystem));

      
    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.intakeNoteOutbutton)
    .onTrue(new IntakeNoteOutCommand(m_IntakeSubsystem));

    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.intakeNoteOutbutton)
    .onFalse(new IntakeWheelsOffCommand(m_IntakeSubsystem));


    // Drive subsystem zero gyro, field centric.
    new JoystickButton(driverJoystick,Constants.DriverJoystick.zeroGyroButton)
      .onTrue(new ZeroGyroCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick,Constants.DriverJoystick.toggleFeildCentricButton)
      .onTrue(new FieldCentricCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick,Constants.DriverJoystick.toggleRobotCentricButton)
      .onTrue(new RobotCentricCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick,Constants.DriverJoystick.driveForwardButton)
      .onTrue(new DriveForwardCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick, Constants.DriverJoystick.reverseDrive)
      .onTrue(new ReverseDriveCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick, Constants.DriverJoystick.unReverseDrive)
      .onTrue(new UnReverseDriveCommand(m_SwerveSubsystem));

      //TODO: Add lock and unlock climber buttons
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }

  public Command getClimberHomingCommand() {
    return new HomeClimberCommand(m_ClimberSubsystem);
  }


  public Command getIntakeHomingCommand() {
    return new HomeIntakeCommand(m_IntakeSubsystem);
  }

  
}
