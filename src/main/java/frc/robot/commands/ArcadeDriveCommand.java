// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final double speedFunction, turnFunction;
  /** Creates a new ArcadeDriveCommand. */
  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, double speedFunction, double turnFunction) {
    this.driveSubsystem = driveSubsystem;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setMotor(speedFunction, turnFunction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}