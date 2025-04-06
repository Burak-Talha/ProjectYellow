package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetReef {

    public int aprilTagId;
    public Pose2d pose2d;
    public TargetReef(int aprilTagId, Pose2d pose2d) {
        //TODO Auto-generated constructor stub
        this.aprilTagId = aprilTagId;
        this.pose2d = pose2d;
    }
}
