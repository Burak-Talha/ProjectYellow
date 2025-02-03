package frc.robot.lib.util.targets;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer.TargetAB;

public class TargetReef {
    public int aprilTagId;
    public TargetAB targetAB;
    public Pose2d pose2d;
    public TargetReef(int aprilTagId, TargetAB targetAB, Pose2d pose2d) {
        //TODO Auto-generated constructor stub
        this.aprilTagId = aprilTagId;
        this.targetAB = targetAB;
        this.pose2d = pose2d;
    }
}
