package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetApril {
    public int aprilTagId;
    public Pose2d aprilTagPose;
    public TargetApril(int apriltagId, Pose2d aprilTagPose){
        this.aprilTagId = apriltagId;
        this.aprilTagPose = aprilTagPose;
    }
}
