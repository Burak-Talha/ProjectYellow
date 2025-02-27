// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.TargetAB;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToHumanIntakeCommand extends Command {

  SwerveSubsystem swerveSubsystem;
  String targetType;
  TargetAB targetAB;

  /** Creates a new GoToTargetCommand. */
  public GoToHumanIntakeCommand(SwerveSubsystem swerveSubsystem, TargetAB targetAB) {
    this.swerveSubsystem = swerveSubsystem;
    this.targetAB = targetAB;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  /** Creates a new GoToHumanIntakeCommand. */
  public GoToHumanIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentTarget = null;
    try{
    if(targetAB == TargetAB.A){
      currentTarget = swerveSubsystem.getHumanPlayerABTarget()[0];
    }
    else if(targetAB == TargetAB.B){
      currentTarget = swerveSubsystem.getHumanPlayerABTarget()[1];
    }

    SmartDashboard.putNumber("HUMAN TARGET POSE X:", currentTarget.getX());
    SmartDashboard.putNumber("HUMAN TARGET POSE Y:", currentTarget.getY());
    SmartDashboard.putNumber("HUMAN TARGET POSE ROTATION:", currentTarget.getRotation().getDegrees());

    double Xerror = currentTarget.getX() - swerveSubsystem.getEstimatedPose().getX();
    double Yerror = currentTarget.getY() - swerveSubsystem.getEstimatedPose().getY();
    double zError = swerveSubsystem.getGyroAngle() - currentTarget.getRotation().getDegrees();
    double kP = 0.1;
    zError = (zError + 180) % 360 - 180;
    SmartDashboard.putNumber("Z ERROR:", zError);
   swerveSubsystem.drive(new Translation2d(-Xerror,-Yerror*1.2), -zError*kP, true, true);
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
