// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CleanerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CleanerSubsystem;
import frc.robot.subsystems.CleanerSubsystem.DesiredCleanerPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateCleanerCommand extends Command {

  CleanerSubsystem cleanerSubsystem;
  DesiredCleanerPosition desiredCleanerPosition;

  /** Creates a new IntakeRotateDefaultCommand. */
  public RotateCleanerCommand(CleanerSubsystem cleanerSubsystem, DesiredCleanerPosition desiredCleanerPosition) {
    this.cleanerSubsystem = cleanerSubsystem;
    this.desiredCleanerPosition = desiredCleanerPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cleanerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cleanerSubsystem.calculateProfiledDemand(desiredCleanerPosition);
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      cleanerSubsystem.stopMotors();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return cleanerSubsystem.atSetpoint();
    }
}
