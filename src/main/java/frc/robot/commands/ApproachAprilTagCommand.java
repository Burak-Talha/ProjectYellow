package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Rotation2d;

public class ApproachAprilTagCommand extends Command {

    private final Swerve swerveSubsystem;
    private final Vision visionSubsystem;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private final int targetId;
    private Pose2d robotPose;
    private Pose2d targetPose;
    private static final double DESIRED_DISTANCE = 1.0; // 1 metre yaklaşma mesafesi
    private static final double ALIGNMENT_TOLERANCE = 0.2; // X ve Y ekseninde 20 cm tolerans
    private static final double ANGLE_TOLERANCE = Math.toRadians(5); // 5 derece tolerans

    PIDController xPidController = new PIDController(0.5, 0, 0);
    PIDController yPidController = new PIDController(0.5, 0, 0);
    PIDController thetaPidController = new PIDController(0.05, 0, 0);
    

    public ApproachAprilTagCommand(Swerve swerveSubsystem, Vision visionSubsystem, int targetId) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetId = targetId;

        addRequirements(swerveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        if (targetPose == null) {
            System.out.println("AprilTag " + targetId + " pozisyonu bulunamadı!");
        } else {
            System.out.println("Hedef AprilTag Pozisyonu: " + targetPose);
        }
    }

    @Override
    public void execute() {
        // Hedef AprilTag'in pozisyonunu al
        targetPose = visionSubsystem.getAprilTagPose(targetId);
        robotPose = swerveSubsystem.getPose();
        double x = xPidController.calculate(robotPose.getX(), targetPose.getX());
        double y = yPidController.calculate(robotPose.getY(), targetPose.getY()-1);
        double degree = thetaPidController.calculate(visionSubsystem.getTheDegreeBetweenRobotAndApril(), 0);
        int detectedId = visionSubsystem.getDetectedTagId();
        if (detectedId != targetId) {
            System.out.println("Hedef ID algılanmadı! Algılanan ID: " + detectedId);
            swerveSubsystem.stop();
            return; // Hedef ID algılanmazsa hareket durur
        }

        // Limelight'tan alınan robot pozisyonunu al
        Pose2d currentPose = visionSubsystem.getRobotPose();

        if (currentPose == null) {
            System.out.println("Mevcut robot pozisyonu alınamadı!");
            swerveSubsystem.stop();
            return;
        }
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, -degree);
        swerveSubsystem.driveRobotRelative(chassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) {
            return true; // Geçersiz hedef varsa komutu hemen bitir
        }

        // Robot belirlenen mesafeye ulaştığında ve açı hizalandığında komut bitir
        Pose2d currentPose = visionSubsystem.getRobotPose();

        if (currentPose == null) {
            return true; // Pozisyon alınamıyorsa komutu bitir
        }

        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        return distanceToTarget <= DESIRED_DISTANCE && isAlignedWithTarget(currentPose, targetPose) && isAngleAligned(currentPose, targetPose);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("ApproachAprilTagCommand kesintiye uğradı.");
        } else {
            System.out.println("Robot hedefe başarıyla yaklaştı: " + targetPose);
        }

        // Hareketi durdur
        swerveSubsystem.stop();
    }

    private boolean isAlignedWithTarget(Pose2d currentPose, Pose2d targetPose) {
        double deltaX = Math.abs(targetPose.getX() - currentPose.getX());
        double deltaY = Math.abs(targetPose.getY() - currentPose.getY());
        return deltaX < ALIGNMENT_TOLERANCE && deltaY < ALIGNMENT_TOLERANCE;
    }

    private boolean isAngleAligned(Pose2d currentPose, Pose2d targetPose) {
        double angleError = Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());
        return angleError < ANGLE_TOLERANCE;
    }
}
