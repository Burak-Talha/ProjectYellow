// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer.TargetAB;
import frc.robot.subsystems.SwerveSubsystem;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToTargetCommand extends Command {

  enum TargetType{
    HumanIntake,
    DeployReef,
    CoProcessor
  }

  SwerveSubsystem swerveSubsystem;
  String targetType;
  TargetAB targetAB;
  PS5Controller joystick;

  /** Creates a new GoToTargetCommand. */
  public GoToTargetCommand(SwerveSubsystem swerveSubsystem, String targetType, TargetAB targetAB) {
    this.swerveSubsystem = swerveSubsystem;
    this.targetType = targetType;
    this.targetAB = targetAB;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  } 

  Pose2d currentTarget;

  @Override
  public void execute() {
    // we will assign this "swerveSubsystem.getCurrentTargetPose();" when we're ready to go

    // Approach to target by nearest pose data
    /*if(targetType==TargetType.HumanIntake.name()){
    currentTarget = swerveSubsystem.getHumanPlayerTarget();
    }else if(targetType==TargetType.DeployReef.name()){
    currentTarget = swerveSubsystem.getDeployReefTarget();
    }*/

    // Approach to target by A, B position input
try{
  if(targetType==TargetType.HumanIntake.name()){
    if(targetAB == TargetAB.A){
      currentTarget = swerveSubsystem.getHumanPlayerABTarget()[0];
    }else if(targetAB == TargetAB.B){
      currentTarget = swerveSubsystem.getHumanPlayerABTarget()[1];
    }
  }
  else if(targetType==TargetType.DeployReef.name()){
    if(targetAB == TargetAB.A){
      //currentTarget = swerveSubsystem.getDeployReefTarget().get(0).pose2d;
    }else if(targetAB == TargetAB.B){
      //currentTarget = swerveSubsystem.getDeployReefTarget().get(1).pose2d;
    }
  }
  else if(targetType==TargetType.CoProcessor.name()){
      currentTarget = FieldConstants.redCoProcessorTarget;
  }

    double Xerror = currentTarget.getX() - swerveSubsystem.getEstimatedPose().getX();
    double Yerror = currentTarget.getY() - swerveSubsystem.getEstimatedPose().getY();
    double zError = swerveSubsystem.getGyroAngle() - currentTarget.getRotation().getDegrees();
    double kP = 0.1;
    zError = (zError + 180) % 360 - 180;
   swerveSubsystem.drive(new Translation2d(Xerror*1,Yerror*1), zError*kP, true, true);}
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
