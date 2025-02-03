// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.TargetAB;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToCoProcessorCommand extends Command {

  Swerve swerveSubsystem;

  PIDController xPidController = new PIDController(1.2, 0.001, 0.01);
  PIDController yPidController = new PIDController(1.2, 0.046, 0);

  public GoToCoProcessorCommand(Swerve swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentTarget = null;
    try{
    if(DriverStation.getAlliance().get()==Alliance.Red){
      currentTarget = FieldConstants.RED_COPROCESSOR_TARGET;
    } else{
      currentTarget = FieldConstants.BLUE_COPROCESSOR_TARGET;
    }

    double xDemand = xPidController.calculate(swerveSubsystem.getEstimatedPose().getX(), currentTarget.getX());
    double yDemand = yPidController.calculate(swerveSubsystem.getEstimatedPose().getY(), currentTarget.getY());
    double zDemand = swerveSubsystem.getGyroAngle() - currentTarget.getRotation().getDegrees();
    zDemand = (zDemand + 180) % 360 - 180;
    
    swerveSubsystem.drive(new Translation2d(-xDemand*1,-yDemand*1), -zDemand*0.1, true, true);
  }
   catch(Exception exception){
    exception.printStackTrace();
   }
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
