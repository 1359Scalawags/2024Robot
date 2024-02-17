// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FeildCentricDrive extends Command {
    private SwerveSubsystem swerve;
    private DoubleSupplier vX;
    private DoubleSupplier vY;
    private DoubleSupplier omega;
    private DoubleSupplier throttle;
    private BooleanSupplier feildRelitive;
    private boolean isOpenLoop;
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
    public FeildCentricDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, DoubleSupplier throttle, BooleanSupplier feildRelitive, boolean isOpenLoop) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.throttle = throttle;
        this.feildRelitive = feildRelitive;
        this.isOpenLoop = isOpenLoop;
        this.controller = swerve.getSwerveController();
        addRequirements(swerve);
    }



    @Override
    public void execute() {
        double modvX = vX.getAsDouble();
        double modvY = vY.getAsDouble();
        if(Math.abs(vX.getAsDouble()) < Constants.swereSubsystem.kTeleopDeadzone && Math.abs(vY.getAsDouble()) < Constants.swereSubsystem.kTeleopDeadzone) {
            modvX = 0;
            modvY = 0;
        }
        //Translation2d translation, double rotation, boolean fieldRelative
        double xVelocity = (modvX * swerve.getMaxSpeed()) * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1);
        double yVelocity = (modvY * swerve.getMaxSpeed()) * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1);
        double angVelocity = (Math.pow(MathUtil.applyDeadband(omega.getAsDouble(), 0.2), 3) * controller.config.maxAngularVelocity) * 0.5;
        swerve.drive(
            new Translation2d(xVelocity, yVelocity),
            angVelocity,
            feildRelitive.getAsBoolean());

        SmartDashboard.putNumber("Throttle", throttle.getAsDouble() * 100);
        SmartDashboard.putNumber("swerve X", modvX);
        SmartDashboard.putNumber("swerve Y", modvY);

    }

}
