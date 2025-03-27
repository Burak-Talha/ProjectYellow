// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.ElevatorSubsytem.DesiredElevatorPosition;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RaiseElevatorCommand extends InstantCommand {

  private ElevatorSubsytem elevatorSubsystem;
  private DesiredElevatorPosition desiredElevatorPosition;

  public RaiseElevatorCommand(ElevatorSubsytem elevatorSubsystem, DesiredElevatorPosition desiredElevatorPosition) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.desiredElevatorPosition = desiredElevatorPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("BEN calisiyorum ", 0);
    elevatorSubsystem.setDesiredElevatorPosition(desiredElevatorPosition);
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      SmartDashboard.putBoolean("RAISE ELEVATOR HAS ENDED", true);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return elevatorSubsystem.atSetpoint();
    }

}
