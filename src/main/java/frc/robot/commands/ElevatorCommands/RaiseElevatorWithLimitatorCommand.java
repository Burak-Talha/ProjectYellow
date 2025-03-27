// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.ElevatorSubsytem.DesiredElevatorPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseElevatorWithLimitatorCommand extends Command {

  private ElevatorSubsytem elevatorSubsystem;
  private DesiredElevatorPosition desiredElevatorPosition;

  public RaiseElevatorWithLimitatorCommand(ElevatorSubsytem elevatorSubsystem, DesiredElevatorPosition desiredElevatorPosition) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.desiredElevatorPosition = desiredElevatorPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setDesiredElevatorPosition(desiredElevatorPosition);
  }
}
