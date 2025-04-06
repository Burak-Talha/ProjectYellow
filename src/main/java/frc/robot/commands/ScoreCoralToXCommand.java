// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ElevatorSubsytem.DesiredElevatorPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoralToXCommand extends Command {

  private GripperSubsystem gripperSubsystem;
  private ElevatorSubsytem elevatorSubsytem;
  private DesiredElevatorPosition desiredElevatorPosition;
  Timer timer = new Timer();

  /** Creates a new ScoreCoralToL1Command. */
  public ScoreCoralToXCommand(ElevatorSubsytem elevatorSubsytem, GripperSubsystem gripperSubsystem, DesiredElevatorPosition desiredElevatorPosition) {
    this.gripperSubsystem = gripperSubsystem;
    this.elevatorSubsytem = elevatorSubsytem;
    this.desiredElevatorPosition = desiredElevatorPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsytem.setDesiredElevatorPosition(desiredElevatorPosition);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("ELEVATOR AT SETPOINT: ", elevatorSubsytem.atSetpoint());
    if(elevatorSubsytem.atSetpoint){
      gripperSubsystem.getOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.stopMotors();
    //elevatorSubsytem.setDesiredElevatorPosition(DesiredElevatorPosition.L1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 2.5;
  }
}
