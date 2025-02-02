package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.RobotContainer.TargetAB;
import frc.robot.lib.util.TargetReef;

public class FieldConstants {
    public static final Pose2d[] REEF_APRIL_TAG_POSE = {
        new Pose2d(4.073905999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(240.0))),
        new Pose2d(3.6576, 4.0259, new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(4.073905999999999, 4.745482, new Rotation2d(Math.toRadians(120.0))),
        new Pose2d(4.904739999999999, 4.745482, new Rotation2d(Math.toRadians(60.0))),
        new Pose2d(5.321046, 4.0259, new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(4.904739999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(300.0))),
        new Pose2d(13.474446, 3.3063179999999996, new Rotation2d(Math.toRadians(300.0))),
        new Pose2d(13.890498, 4.0259, new Rotation2d(Math.toRadians(0.0))),
        new Pose2d(13.474446, 4.745482, new Rotation2d(Math.toRadians(60.0))),
        new Pose2d(12.643358, 4.745482, new Rotation2d(Math.toRadians(120.0))),
        new Pose2d(12.227305999999999, 4.0259, new Rotation2d(Math.toRadians(180.0))),
        new Pose2d(12.643358, 3.3063179999999996, new Rotation2d(Math.toRadians(240.0)))
      };
  
      public static final Pose2d[] redHumanPlayerTargets = {
          new Pose2d(16.5, 0.944, new Rotation2d(Math.toRadians(50))),
          new Pose2d(16.7, 7, new Rotation2d(Math.toRadians(-50)))
      };
  
      public static final Pose2d[] blueHumanPlayerTargets = {
          new Pose2d(1.112, 1.065, new Rotation2d(Math.toRadians(50))),
          new Pose2d(1.322, 6.955, new Rotation2d(Math.toRadians(-15)))
      };
  
      public static final Pose2d redCoProcessorTarget = new Pose2d(11.7, 7.4  , new Rotation2d(Math.toRadians(-90)));
      public static final Pose2d blueCoProcessorTarget = new Pose2d(11.6, 7.6  , new Rotation2d(Math.toRadians(0)));
  
      public static final TargetReef[] BLUE_TARGET_REEFS = {};
      public static final TargetReef[] RED_TARGET_REEFS = {
          new TargetReef(7, TargetAB.A, new Pose2d(13.9, 3.865, new Rotation2d(Math.toRadians(0)))),
          new TargetReef(7, TargetAB.B, new Pose2d(13.9, 4.2, new Rotation2d(Math.toRadians(0)))),
          new TargetReef(8, TargetAB.A, new Pose2d(13.59, 4.7, new Rotation2d(Math.toRadians(-60)))),
          new TargetReef(8, TargetAB.B, new Pose2d(13.35, 4.872, new Rotation2d(Math.toRadians(-60)))),
          new TargetReef(9, TargetAB.A, new Pose2d(12.46, 5.2, new Rotation2d(Math.toRadians(-120)))),
          new TargetReef(9, TargetAB.B, new Pose2d(12, 4.982, new Rotation2d(Math.toRadians(-120)))),
          new TargetReef(10, TargetAB.A, new Pose2d(12.186, 3.871, new Rotation2d(Math.toRadians(180)))),
          new TargetReef(10, TargetAB.B, new Pose2d(12.186, 4.193, new Rotation2d(Math.toRadians(180)))),
          new TargetReef(11, TargetAB.A, new Pose2d(12.7, 3.4, new Rotation2d(Math.toRadians(120)))),
          new TargetReef(11, TargetAB.B, new Pose2d(12.3, 3.7, new Rotation2d(Math.toRadians(120)))),
          new TargetReef(6, TargetAB.A, new Pose2d(13.45, 3.2, new Rotation2d(Math.toRadians(60)))),
          new TargetReef(6, TargetAB.B, new Pose2d(13.65, 3.26, new Rotation2d(Math.toRadians(60))))
      };
  
      public static final Translation2d kLeftTargetOffset = new Translation2d(0.5, -0.164);
      public static final Translation2d kRightTargetOffset = new Translation2d(0.5, 0.164);
      public static final Translation2d kCenterTargetOffset = new Translation2d(0, 0.0);
  
      public static final Pose2d[] kReefBlueApriltags = {
        new Pose2d(4.073905999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(60))),
        new Pose2d(3.6576, 4.0259, new Rotation2d(Math.toRadians(0))),
        new Pose2d(4.073905999999999, 4.745482, new Rotation2d(Math.toRadians(-60))),
        new Pose2d(4.904739999999999, 4.745482, new Rotation2d(Math.toRadians(-120))),
        new Pose2d(5.321046, 4.0259, new Rotation2d(Math.toRadians(180))),
        new Pose2d(4.904739999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(120))),
      };
  
      public static final Pose2d[] kReefRedApriltags = {
          new Pose2d(13.4, 3.2, new Rotation2d(Math.toRadians(60))),
          new Pose2d(13.9, 4.055, new Rotation2d(Math.toRadians(0))),
          new Pose2d(13.478 +0.3, 4.776 +0.3, new Rotation2d(Math.toRadians(-60))),
          new Pose2d(12.652, 4.776, new Rotation2d(Math.toRadians(-120))),
          new Pose2d(12.216, 4.055, new Rotation2d(Math.toRadians(180))),
          new Pose2d(12.652, 3.319, new Rotation2d(Math.toRadians(120)))
      };
    }