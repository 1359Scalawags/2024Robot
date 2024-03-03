// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoCommunity2Command extends Command {
    private SwerveSubsystem swerve;
    private SwerveController controller;

    private Timer timer;
    /**
     * 
     * @param swerve
     * @param vX double between -1, and 1
     * @param vY double between -1, and 1
     * @param omega
     * @param throttle
     * @param feildRelitive
     * @param isOpenLoop
     */
    public AutoCommunity2Command(SwerveSubsystem swerve) {
        timer = new Timer();
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }


    @Override
    public void execute() {
      
        //Translation2d translation, double rotation, boolean fieldRelative
       
        swerve.drive(
            new Translation2d(0.0, 0.5),
            0.0,
            false);
    }
    @Override
    public void end(boolean interrupted) {
        swerve.drive(
            new Translation2d(0.0, 0.0),
            0.0,
            false);
    }
    @Override
    public boolean isFinished() {
        if (timer.get() >= Constants.swerveSubsystem.CommunityAutoTime) {
            return true;
        }
        else return false;
    }
}
