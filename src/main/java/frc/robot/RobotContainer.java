// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FieldDrive;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  XboxController driver = new XboxController(0);
  DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new FieldDrive(
      driveSubsystem,
      () -> driver.getLeftY() * 4.2,
      () -> driver.getLeftX() * 4.2,
      () -> driver.getRightX() * 2 * Math.PI));  

    configureBindings();
  }

  private void configureBindings() {
    JoystickButton jsb = new JoystickButton(driver, 4);
    jsb.whileTrue(new InstantCommand(driveSubsystem::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
