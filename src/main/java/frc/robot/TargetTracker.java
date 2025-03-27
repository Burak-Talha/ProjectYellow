package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveSubsystem;

public class TargetTracker {

    private SwerveSubsystem swerveSubsystem;

    public Pose2d currentPosition;
    public Pose2d currentTarget;

    public TargetTracker(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }

    
    
}
