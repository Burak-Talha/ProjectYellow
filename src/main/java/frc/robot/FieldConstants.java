package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.util.TargetReef;

public class FieldConstants {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Translation2d LEFT_TARGET_OFFSET = new Translation2d(0.38, -0.22);
    public static final Translation2d RIGHT_TARGET_OFFSET = new Translation2d(0.38, 0.16);
    public static final Translation2d CENTER_TARGET_OFFSET = new Translation2d(0.38, 0);
    }