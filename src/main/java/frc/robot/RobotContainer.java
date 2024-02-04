// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveCommands.FieldCentricCommand;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.commands.ExtendArmCommand;
// import frc.robot.commands.RetractArmCommand;
// import frc.robot.commands.ShootCommand;
// import frc.robot.commands.StopShootingCommand;
// import frc.robot.commands.IntakeCommands.IntakeBeltOffCommand;
// import frc.robot.commands.IntakeCommands.IntakeBeltOnCommand;
// import frc.robot.commands.IntakeCommands.IntakeExtendCommand;
// import frc.robot.commands.IntakeCommands.IntakeRetractCommand;
// import frc.robot.commands.IntakeCommands.IntakeWheelsOffCommand;
// import frc.robot.commands.IntakeCommands.IntakeWheelsOnCommand;
import frc.robot.commands.SwerveCommands.FeildCentricDrive;
import frc.robot.commands.SwerveCommands.ZeroGyroCommand;
// import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
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
    new File(Filesystem.getDeployDirectory(), "YAGSLConfigJSON/swerve"));
  // private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  // private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick assistantJoystick = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands(); 

  }
    /*
     * SwerveSubsystem swerve,
     *  DoubleSupplier vX, 
     * DoubleSupplier vY, 
     * DoubleSupplier omega,
     *  DoubleSupplier throttle, 
     * BooleanSupplier feildRelitive,
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
  
    // new JoystickButton(driverJoystick,Constants.DriverJoystick.shootButton)
    //   .onTrue(new ShootCommand(m_shooterSubsystem));
    
    // new JoystickButton(driverJoystick,Constants.DriverJoystick.shootButton)
    //   .onFalse(new StopShootingCommand(m_shooterSubsystem));
    // //Shooter commands/binds above

    // new JoystickButton(driverJoystick,Constants.DriverJoystick.extendArmButton)
    //   .onTrue(new ExtendArmCommand(m_ClimberSubsystem));

    // new JoystickButton(driverJoystick,Constants.DriverJoystick.retractArmButton)
    //   .onTrue(new RetractArmCommand(m_ClimberSubsystem));
    // //Climber commands/binds above

    // new JoystickButton(driverJoystick,Constants.DriverJoystick.intakeBeltButton)
    //   .onTrue(new IntakeBeltOnCommand(m_IntakeSubsystem));

    // new JoystickButton(driverJoystick,Constants.DriverJoystick.intakeBeltButton)
    //   .onFalse(new IntakeBeltOffCommand(m_IntakeSubsystem));

    // new JoystickButton(driverJoystick,Constants.DriverJoystick.intakeExtendButton)
    //   .onTrue(new IntakeExtendCommand(m_IntakeSubsystem));

    // new JoystickButton(driverJoystick,Constants.DriverJoystick.intakeExtendButton)
    //   .onFalse(new IntakeRetractCommand(m_IntakeSubsystem));

    // new JoystickButton(driverJoystick,Constants.DriverJoystick.intakeWheelsOnbutton)
    //   .onTrue(new IntakeWheelsOnCommand(m_IntakeSubsystem));

    //   new JoystickButton(driverJoystick,Constants.DriverJoystick.intakeWheelsOnbutton)
    //   .onFalse(new IntakeWheelsOffCommand(m_IntakeSubsystem));
    // //intake commands/binds above


    //Drive subsystem zero gyro, field centric.







  
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    new JoystickButton(driverJoystick,Constants.DriverJoystick.zeroGyroButton)
      .onTrue(new ZeroGyroCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick,Constants.DriverJoystick.toggleFeildCentricButton)
      .onTrue(new FieldCentricCommand(m_SwerveSubsystem));

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

}
