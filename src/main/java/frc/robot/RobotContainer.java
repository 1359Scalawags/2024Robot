// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveCommands.FieldCentricCommand;
import frc.robot.Constants.swerveSubsystem;
import frc.robot.commands.SetDefaultPipelineCommand;
import frc.robot.commands.ArmCommands.ExtendArmCommand;
import frc.robot.commands.ArmCommands.HomeClimberCommand;
import frc.robot.commands.ArmCommands.LockClimberCommand;
import frc.robot.commands.ArmCommands.MoveClimberArms;
import frc.robot.commands.ArmCommands.RetractArmCommand;
import frc.robot.commands.ArmCommands.UnlockClimberCommand;
import frc.robot.commands.IntakeCommands.IntakeWheelsOffCommand;
import frc.robot.commands.IntakeCommands.TimedCommands.IntakeAmpTimedShoot;
import frc.robot.commands.IntakeCommands.TimedCommands.IntakeNoteInTimedShoot;
import frc.robot.commands.IntakeCommands.TimedCommands.IntakeNoteOutTimedShoot;
import frc.robot.commands.IntakeCommands.TimedCommands.SecondIntakeNoteOutTimedShoot;
import frc.robot.commands.IntakeCommands.IntakeNoteInCommand;
import frc.robot.commands.IntakeCommands.IntakeNoteOutCommand;
import frc.robot.commands.IntakeCommands.HomeIntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeExtendAmpCommand;
import frc.robot.commands.IntakeCommands.IntakeExtendCommand;
import frc.robot.commands.IntakeCommands.IntakeRetractCommand;
import frc.robot.commands.IntakeCommands.IntakeWheelsOffCommand;
import frc.robot.commands.IntakeCommands.IntakeNoteInCommand;
import frc.robot.commands.ShootingCommands.AmpShootCommand;
import frc.robot.commands.ShootingCommands.SecondShootTimedCommand;
import frc.robot.commands.ShootingCommands.ShootCommand;
import frc.robot.commands.ShootingCommands.ShootTimedCommand;
import frc.robot.commands.ShootingCommands.StopAmpShootingCommand;
import frc.robot.commands.ShootingCommands.StopShootingCommand;
import frc.robot.commands.SwerveCommands.AutoCommunity1Command;
import frc.robot.commands.SwerveCommands.AutoCommunity2Command;
import frc.robot.commands.SwerveCommands.DriveForwardCommand;
import frc.robot.commands.SwerveCommands.DriveRightCommand;
import frc.robot.commands.SwerveCommands.FeildCentricDrive;
import frc.robot.commands.SwerveCommands.FieldCentricCommand;
import frc.robot.commands.SwerveCommands.ReverseDriveCommand;
import frc.robot.commands.SwerveCommands.RobotCentricCommand;
import frc.robot.commands.SwerveCommands.RotateCCWCommand;
import frc.robot.commands.SwerveCommands.UnReverseDriveCommand;
import frc.robot.commands.SwerveCommands.ZeroGyroCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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


SendableChooser<Command> autoChooser;
    
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "YAGSLConfigJSON/swerve/" + Constants.robotName));
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final Joystick driverJoystick = new Joystick(Constants.DriverJoystick.joystick);
  private final Joystick assistantJoystick = new Joystick(Constants.AssistantJoystick.joystick);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // this is how you add commands to be used in auto routeine
    NamedCommands.registerCommand("ShootCommand", new ShootCommand(m_shooterSubsystem));
    NamedCommands.registerCommand("ShootTimedCommand", new ShootTimedCommand(m_shooterSubsystem));
    NamedCommands.registerCommand("IntakeNoteOutTimedShoot", new IntakeNoteOutTimedShoot(m_IntakeSubsystem));
    NamedCommands.registerCommand("IntakeExtendCommand", new IntakeExtendCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("IntakeNoteInTimedShoot", new IntakeNoteInTimedShoot(m_IntakeSubsystem));
    NamedCommands.registerCommand("IntakeRetractCommand", new IntakeRetractCommand(m_IntakeSubsystem));
    NamedCommands.registerCommand("AutoCommunity2Command", new AutoCommunity2Command(m_SwerveSubsystem));
    NamedCommands.registerCommand("AutoCommunity1Command", new AutoCommunity1Command(m_SwerveSubsystem));
    NamedCommands.registerCommand("SecondShootTimedCommand", new SecondShootTimedCommand(m_shooterSubsystem));
    NamedCommands.registerCommand("SecondIntakeNoteOutTimedShoot", new SecondIntakeNoteOutTimedShoot(m_IntakeSubsystem));


    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands(); 

    autoChooser = AutoBuilder.buildAutoChooser();

    // autoChooser.addOption("Test Auto", getAutonomousCommand("Test Auto"));
    autoChooser.addOption("Test Auto Two", getAutonomousCommand("Test Auto Two"));
    autoChooser.addOption("Shooting Auto", getAutonomousCommand("Shooting Auto"));
    autoChooser.addOption("Nothing Auto", getAutonomousCommand("Nothing Auto"));
    autoChooser.addOption("Basic Pos 2", getAutonomousCommand("Basic Pos 2"));
    autoChooser.addOption("Basic Pos 3", getAutonomousCommand("Basic Pos 3"));
    autoChooser.addOption("Basic Pos 1", getAutonomousCommand("Basic Pos 1"));
    autoChooser.addOption("Two Note Auto pos 2", getAutonomousCommand("Two Note Auto pos 2"));
    //autoChooser.addOption("MoveOnly", getAutonomousCommand("MoveOnly"));
    //autoChooser.addOption("MoveAndShoot", getAutonomousCommand("MoveAndShoot"));
    //autoChooser.addOption("Test Auto Two", getAutonomousCommand("Test Auto Two"));
    //autoChooser.addOption("Test Auto", getAutonomousCommand("Test Auto"));
    //autoChooser.addOption("Two Note Auto pos 2", getAutonomousCommand("Two Note Auto pos 2"));


    //autoChooser.addOption("Example Path", Path("example Path"));
    //autoChooser.addOption("New Auto", Auto("New Auto"));
     SmartDashboard.putData("Auto Chooser ", autoChooser);
    // SmartDashboard.putData("Second Chooser ", autoChooser);
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
      this::driverGetForward,
      this::driverGetRight,
      this::driverGetZ,
      this::driverGetThrottle,
      m_SwerveSubsystem::getFeildCentric, 
      false
      ));

    m_ClimberSubsystem.setDefaultCommand(
      new MoveClimberArms(m_ClimberSubsystem,
      this::assistantGetY
      ));
    // m_VisionSubsystem.setDefaultCommand() {
    //   new setDefaultPipeline();
    // };
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
  public double driverGetRight() {
    return -driverJoystick.getX();
  }

  public double driverGetForward() {
    return -driverJoystick.getY();
  }

  public double driverGetZ() {
    return -driverJoystick.getZ();
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
    // new JoystickButton(assistantJoystick,Constants.AssistantJoystick.shootButton)
    //   .onTrue(new ShootCommand(m_shooterSubsystem));
   
    // new JoystickButton(assistantJoystick,Constants.AssistantJoystick.shootButton)
    //   .onFalse(new StopAmpShootingCommand(m_shooterSubsystem));

    new JoystickButton(assistantJoystick, Constants.AssistantJoystick.ampShootingButton)
      .onTrue(new AmpShootCommand(m_shooterSubsystem));
    
    new JoystickButton(assistantJoystick, Constants.AssistantJoystick.ampShootingButton)
      .onFalse(new StopAmpShootingCommand(m_shooterSubsystem));


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

    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.lockClimberButton)
    .onTrue(new LockClimberCommand(m_ClimberSubsystem));

    new JoystickButton(assistantJoystick,Constants.AssistantJoystick.unlockClimberButtom)
    .onTrue(new UnlockClimberCommand(m_ClimberSubsystem));

    // new JoystickButton(assistantJoystick,Constants.AssistantJoystick.ampShootingButton)
    // .onTrue(Commands.sequence(new IntakeExtendAmpCommand(m_IntakeSubsystem), new IntakeAmpTimedShoot(m_IntakeSubsystem)));

    // Drive subsystem zero gyro, field centric.
    new JoystickButton(driverJoystick,Constants.DriverJoystick.zeroGyroButton)
      .onTrue(new ZeroGyroCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick,Constants.DriverJoystick.toggleFeildCentricButton)
      .onTrue(new FieldCentricCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick,Constants.DriverJoystick.toggleRobotCentricButton)
      .onTrue(new RobotCentricCommand(m_SwerveSubsystem));
    
    new JoystickButton(assistantJoystick, Constants.AssistantJoystick.shootLoadedNote)
      .onTrue(Commands.parallel(new ShootTimedCommand(m_shooterSubsystem), new IntakeNoteOutTimedShoot(m_IntakeSubsystem)));

    new JoystickButton(driverJoystick, Constants.DriverJoystick.reverseDrive)
      .onTrue(new ReverseDriveCommand(m_SwerveSubsystem));

    new JoystickButton(driverJoystick, Constants.DriverJoystick.unReverseDrive)
      .onTrue(new UnReverseDriveCommand(m_SwerveSubsystem));
      
      // test buttons
    // new JoystickButton(driverJoystick,Constants.DriverJoystick.auto2driveForwardButton)
    //   .onTrue(new AutoCommunity2Command(m_SwerveSubsystem));

    if (Constants.kDebug) {
      new JoystickButton(driverJoystick,Constants.DriverJoystick.driveForwardButton)
        .onTrue(new DriveForwardCommand(m_SwerveSubsystem));

      new JoystickButton(driverJoystick,Constants.DriverJoystick.driveRightButton)
        .onTrue(new DriveRightCommand(m_SwerveSubsystem));

      new JoystickButton(driverJoystick,Constants.DriverJoystick.rotateCCWButton)
        .onTrue(new RotateCCWCommand(m_SwerveSubsystem));    
    }

  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_SwerveSubsystem.zeroGyro();
    return getAutonomousCommandForChooser();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandForChooser() {
    return m_SwerveSubsystem.getAutonomousCommand(autoChooser.getSelected().getName());
  }

// Do i need .getName()?

    public Command getAutonomousCommand(String exampleAuto){
    //return m_SwerveSubsystem.getAutonomousCommand(autoChooser.getSelected().getName());
    return m_SwerveSubsystem.getAutonomousCommand(exampleAuto);
  }
  // public Command Path(String examplePath){
  //   return new PathPlannerAuto(examplePath);
  // }
  public Command getClimberHomingCommand() {
    return new HomeClimberCommand(m_ClimberSubsystem);
  }


  public Command getIntakeHomingCommand() {
    return new HomeIntakeCommand(m_IntakeSubsystem);
  }

  public Command getStartingVisionPipe() {
    return new SetDefaultPipelineCommand(m_VisionSubsystem);
  }
  
}
