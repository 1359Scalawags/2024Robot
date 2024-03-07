// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.extensions.GravityAssistedFeedForward;
import frc.robot.extensions.SendableCANSparkMax;
import frc.robot.extensions.SparkMaxPIDTuner;

public class IntakeSubsystem extends SubsystemBase {

  private SendableCANSparkMax topSushiMotor;
  private SendableCANSparkMax bottomStarMotor;
  private SendableCANSparkMax positionMotor;
  private SparkAbsoluteEncoder absolutePositionEncoder;
  private SparkPIDController positionPID;

  private TrapezoidProfile trapezoidProfile;
  private TrapezoidProfile.State positionSetpoint;  // current state of the position motor
  private TrapezoidProfile.State positionGoal; // goal state of the position motor

  private DigitalInput intakeHomeLimit;

  private GravityAssistedFeedForward gravityFF;

  private SparkMaxPIDTuner positionPIDtuner;
  private RelativeEncoder motorEncoder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    topSushiMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kTopWheelMotorPortID, MotorType.kBrushless);
    bottomStarMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kBottomStarMotorPortID, MotorType.kBrushless);

    topSushiMotor.setInverted(false);
    bottomStarMotor.setInverted(false);

    topSushiMotor.setIdleMode(IdleMode.kCoast);
    bottomStarMotor.setIdleMode(IdleMode.kCoast);

    intakeHomeLimit = new DigitalInput(Constants.intakeSubsystem.kHomeLimitID);

    positionMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kPositionMotorPortID, MotorType.kBrushless);

    positionMotor.restoreFactoryDefaults();
    positionMotor.setInverted(true);
    positionMotor.setIdleMode(IdleMode.kBrake);

    absolutePositionEncoder = positionMotor.getAbsoluteEncoder();
    absolutePositionEncoder.setPositionConversionFactor(Constants.intakeSubsystem.kIntakeConversionFactor);
    absolutePositionEncoder.setZeroOffset(Constants.intakeSubsystem.kPositionEncoderOffset);

    // Get PID controller and initialize its values
    positionPID = positionMotor.getPIDController();
    positionPID.setP(Constants.intakeSubsystem.kIntakeP);
    positionPID.setI(Constants.intakeSubsystem.kIntakeI);
    positionPID.setD(Constants.intakeSubsystem.kIntakeD);
    positionPID.setFF(0);

    //set safe homing range
    positionPID.setOutputRange(-0.3, 0.3);
    positionPID.setFeedbackDevice(absolutePositionEncoder);

    // set the max velocity and max acceleration of the intake arm
    Constraints trapezoidConstraints = new Constraints(Constants.intakeSubsystem.kPositionRateLimit, Constants.intakeSubsystem.kPositionAccelerationLimit);
    trapezoidProfile = new TrapezoidProfile(trapezoidConstraints);
    positionSetpoint = new TrapezoidProfile.State(getpos(), 0);
    
    // don't move at robot initialization
    positionGoal = positionSetpoint;
    
    // instantiate feedfoward provider that adapts to arms position
    gravityFF = new GravityAssistedFeedForward(Constants.intakeSubsystem.kGravityFF, Constants.intakeSubsystem.kOffsetAngle);

    Shuffleboard.getTab("Intake").add("Position", positionMotor);

    // only show these when debugging
    if(Constants.kDebug) {
      Shuffleboard.getTab("Intake").add("Star Motor", bottomStarMotor);
      Shuffleboard.getTab("Intake").add("Sushi Motor", topSushiMotor);
      Shuffleboard.getTab("Intake").add("Position Limitswitch", intakeHomeLimit);
      Shuffleboard.getTab("Intake").addDouble("intake pos", this::getpos);
      positionPIDtuner = new SparkMaxPIDTuner("PID Tuner", "Intake Position Motor", 1, positionPID);
    }

  }

  double getpos() {
    return absolutePositionEncoder.getPosition();
  }

  public void ejectNote() {
    topSushiMotor.set(-Constants.intakeSubsystem.kSushiMotorPushOutSpeed);
    bottomStarMotor.set(-Constants.intakeSubsystem.kStarMotorPushOutSpeed);
  }

  public void ejectNoteToShooter() {
    topSushiMotor.set(-Constants.intakeSubsystem.kSushiMotorFeedShooterSpeed);
    bottomStarMotor.set(-Constants.intakeSubsystem.kStarMotorFeedShooterSpeed);
  }

  public void injectNote() {
    topSushiMotor.set(Constants.intakeSubsystem.kSushiMotorDrawInSpeed);
    bottomStarMotor.set(Constants.intakeSubsystem.kStarMotorDrawInSpeed);
  }

  public void stopNoteMotors() {
    topSushiMotor.set(Constants.intakeSubsystem.kStopNoteMotors);
    bottomStarMotor.set(Constants.intakeSubsystem.kStopNoteMotors);
  }

  public void positionUp() {
    positionGoal = new TrapezoidProfile.State(Constants.intakeSubsystem.kpositionUp, 0); // end at up position with 0 velocity
  }

  public void positionDown() {
    positionGoal = new TrapezoidProfile.State(Constants.intakeSubsystem.kpositionDown, 0); // end at down position with 0 velocity
  }

  public boolean isUp() {
    return Math.abs(absolutePositionEncoder.getPosition()
        - Constants.intakeSubsystem.kpositionUp) < Constants.intakeSubsystem.kPositionTolerance;
  }

  public boolean isDown() {
    return Math.abs(absolutePositionEncoder.getPosition()
        - Constants.intakeSubsystem.kpositionDown) < Constants.intakeSubsystem.kPositionTolerance;

  }

  public void setOutputRange(double min, double max) {
    positionPID.setOutputRange(min, max);
  }

  @Override
  public void periodic() {
    if (!DriverStation.isTest()) {

      if (intakeHomeLimit.get() == Constants.intakeSubsystem.kHomeLimitPressed) {
        positionGoal = new TrapezoidProfile.State(getpos() + 0.5, 0);
      }

      positionSetpoint = trapezoidProfile.calculate(Constants.kDeltaTime, positionSetpoint, positionGoal);

      // Use dynamic feedforward for gravity
      double setpointFF = gravityFF.calculate(absolutePositionEncoder.getPosition());
      positionPID.setReference(positionSetpoint.position, ControlType.kPosition, 0, setpointFF, ArbFFUnits.kVoltage);


      if (Constants.kDebug) {
        //SmartDashboard.putNumber("Intake Target Position", targetPosition);
        SmartDashboard.putNumber("Intake Goal State", positionGoal.position);
        SmartDashboard.putNumber("Intake Actual Position", absolutePositionEncoder.getPosition());
        SmartDashboard.putNumber("Intake Motor Encoder", motorEncoder.getPosition());
        SmartDashboard.putNumber("Intake Motor Output", positionMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake RPM", absolutePositionEncoder.getVelocity());
        //SmartDashboard.putNumber("Intake Gravity FF", FF);
      }

    } else {
      RobotContainer container = Robot.getRobotContainer();
      double joyX = container.assistantGetX();
      positionMotor.set(joyX / 2);
 
      SmartDashboard.putNumber("Intake Actual Position", absolutePositionEncoder.getPosition());
      SmartDashboard.putNumber("Intake Motor Output", positionMotor.getOutputCurrent());
      SmartDashboard.putNumber("Intake Test Joystick", joyX);

    }


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
