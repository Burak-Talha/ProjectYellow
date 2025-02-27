// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CleanerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CleanerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CleanerGetOutCommand extends Command {

  CleanerSubsystem cleaner;

  /** Creates a new CleanerGetOutCommand. */
  public CleanerGetOutCommand(CleanerSubsystem cleaner) {
    this.cleaner = cleaner;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cleaner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cleaner.getOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cleaner.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
