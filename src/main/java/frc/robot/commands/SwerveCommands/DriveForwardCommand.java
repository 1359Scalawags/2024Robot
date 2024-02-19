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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveForwardCommand extends Command {
    private SwerveSubsystem swerve;
    private SwerveController controller;

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
    public DriveForwardCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.controller = swerve.getSwerveController();
        addRequirements(swerve);
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
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
}
