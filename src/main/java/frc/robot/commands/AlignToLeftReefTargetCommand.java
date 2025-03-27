// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToLeftReefTargetCommand extends Command {
   SwerveSubsystem swerveSubsystem;
  VisionSubsystem vision;
  private Translation2d kOffsetTranslation;
  private Pose2d currentTargetPose2d = null;
  private double errorDistance = Integer.MAX_VALUE;

  private PIDController xController = new PIDController(Constants.AlignmentConstants.X_CONTROLLER_P, Constants.AlignmentConstants.X_CONTROLLER_I, Constants.AlignmentConstants.X_CONTROLLER_D);
  private PIDController yController = new PIDController(Constants.AlignmentConstants.Y_CONTROLLER_P, Constants.AlignmentConstants.Y_CONTROLLER_I, Constants.AlignmentConstants.Y_CONTROLLER_D);
  private PIDController zController = new PIDController(Constants.AlignmentConstants.Z_CONTROLLER_P, Constants.AlignmentConstants.Z_CONTROLLER_I, Constants.AlignmentConstants.Z_CONTROLLER_D);

  /** Creates a new GoToReefTargetCommand. */
  public AlignToLeftReefTargetCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem vision) {
    this.swerveSubsystem = swerveSubsystem;
    this.vision = vision;
    this.kOffsetTranslation = FieldConstants.LEFT_TARGET_OFFSET;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }
  int nearestAprilTagId = -1;
   // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double minDistance = Integer.MAX_VALUE;
    List<AprilTag> aprilTags = FieldConstants.APRIL_TAG_FIELD_LAYOUT.getTags();
    SmartDashboard.putNumber("April Tag size:", aprilTags.size());
    for(AprilTag aprilTag : aprilTags){
      double currentDistance = aprilTag.pose.getTranslation().toTranslation2d().getDistance(swerveSubsystem.getEstimatedPose().getTranslation());
      if(currentDistance < minDistance){
        minDistance = currentDistance;
        currentTargetPose2d = aprilTag.pose.toPose2d().transformBy(new Transform2d(kOffsetTranslation, new Rotation2d(Math.PI)));
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    errorDistance = currentTargetPose2d.getTranslation().getDistance(swerveSubsystem.getEstimatedPose().getTranslation());
    try{
      /*double xDemand = xController.calculate(swerveSubsystem.getEstimatedPose().getX(), currentTargetPose2d.getX());
      double yDemand = yController.calculate(swerveSubsystem.getEstimatedPose().getY(), currentTargetPose2d.getY());
      double zDemand = zController.calculate(swerveSubsystem.getEstimatedPose().getRotation().getDegrees(), currentTargetPose2d.getRotation().getDegrees());*/
      double xDemand = currentTargetPose2d.getX() - swerveSubsystem.getEstimatedPose().getX();
      double yDemand = currentTargetPose2d.getY() - swerveSubsystem.getEstimatedPose().getY();
      double zDemand = currentTargetPose2d.getRotation().getDegrees() - swerveSubsystem.getEstimatedPose().getRotation().getDegrees();
      zDemand = (zDemand + 180) % 360 - 180;
      swerveSubsystem.drive(new Translation2d(xDemand*0.8, yDemand*0.8), zDemand*0.08, true, true);
    }catch(Exception exception){
      exception.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("BENİM İŞİM BİTTİ BEYLER", " -SIR GOTOREEFCOMMAND");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (MathUtil.isNear(xError, nearestAprilTagId, aprilTagId));
    return MathUtil.isNear(0, errorDistance, Constants.AlignmentConstants.DISTANCE_ALIGMENT_TOLERANCE);
  }
}
