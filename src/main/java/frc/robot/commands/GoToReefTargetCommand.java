// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.TargetAB;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReefTargetCommand extends Command {

  SwerveSubsystem swerveSubsystem;
  VisionSubsystem vision;
  TargetAB targetAB;
  int aprilTagId = -1;
  

  /** Creates a new GoToReefTargetCommand. */
  public GoToReefTargetCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem vision, TargetAB targetAB) {
    this.targetAB = targetAB;
    this.swerveSubsystem = swerveSubsystem;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }
  int nearestAprilTagId = -1;
   // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nearestAprilTagId = (int) vision.getTid();
    SmartDashboard.putNumber("NEAREST APRIL TAG ID:", nearestAprilTagId);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentTarget = null;
    try{

        if(targetAB == TargetAB.A){
          currentTarget = Arrays.asList(swerveSubsystem.getDeployReefTarget()).stream().filter(s -> s.aprilTagId==nearestAprilTagId).filter(s -> s.targetAB==TargetAB.A).findFirst().get().pose2d;
        }else if(targetAB == TargetAB.B){
          currentTarget = Arrays.asList(swerveSubsystem.getDeployReefTarget()).stream().filter(s -> s.aprilTagId==nearestAprilTagId).filter(s -> s.targetAB==TargetAB.B).findFirst().get().pose2d;
        }
      
      // Demand Calculations
      double Xerror = currentTarget.getX() - swerveSubsystem.getEstimatedPose().getX();
      double Yerror = currentTarget.getY() - swerveSubsystem.getEstimatedPose().getY();
      double zError = swerveSubsystem.getGyroAngle() - currentTarget.getRotation().getDegrees();
      double kP = 0.1;
      zError = (zError + 180) % 360 - 180;
      swerveSubsystem.drive(new Translation2d(-Xerror*1,-Yerror*1), -zError*kP, true, true);
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
