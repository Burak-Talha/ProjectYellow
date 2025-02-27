package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;

public class VisionSubsystem extends SubsystemBase {

    // NetworkTable referansları
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry botpose = limelightTable.getEntry("botpose");
    private final NetworkTableEntry tagIdEntry = limelightTable.getEntry("tid");

    // Robotun sahadaki pozisyonu
    private Pose2d robotPose = new Pose2d();
    // Algılanan AprilTag ID'si
    private int detectedTagId = -1;

    // Tüm AprilTag'lerin sabit saha üzerindeki pozisyonları
    private final Map<Integer, Pose2d> aprilTagMap = new HashMap<>();

    public VisionSubsystem() {
        initializeAprilTags();
    }

    private void initializeAprilTags() {
        //
        aprilTagMap.put(1, new Pose2d(100,  100, yawToRotation2d(-60)));
        aprilTagMap.put(2, new Pose2d(100, 110, yawToRotation2d(-60)));
        aprilTagMap.put(3, new Pose2d(110, 131, yawToRotation2d(-60)));
        aprilTagMap.put(4, new Pose2d(100, 112, yawToRotation2d(-60)));
        aprilTagMap.put(5, new Pose2d(153, 155, yawToRotation2d(-60)));
        //
        aprilTagMap.put(6, new Pose2d(4.700446, -0.719682, yawToRotation2d(-60)));
        aprilTagMap.put(7, new Pose2d(13.9, 4.055, yawToRotation2d(0)));
        aprilTagMap.put(8, new Pose2d(13.478, 4.776, yawToRotation2d(60)));
        aprilTagMap.put(9, new Pose2d(12.652, 4.776, yawToRotation2d(120)));
        aprilTagMap.put(10, new Pose2d(12.216, 4.055, yawToRotation2d(180)));
        aprilTagMap.put(11, new Pose2d(12.652, 3.319, yawToRotation2d(-120)));
        //
        aprilTagMap.put(12, new Pose2d(100, 112, yawToRotation2d(-60)));
        aprilTagMap.put(13, new Pose2d(100, 112, yawToRotation2d(-60)));
        aprilTagMap.put(14, new Pose2d(100, 112, yawToRotation2d(-60)));
        aprilTagMap.put(15, new Pose2d(100, 112, yawToRotation2d(-60)));
        aprilTagMap.put(16, new Pose2d(100, 112, yawToRotation2d(-60)));
        //
        aprilTagMap.put(17, new Pose2d(4.087, 3.289, yawToRotation2d(-120)));
        aprilTagMap.put(18, new Pose2d(3.636, 4.040, yawToRotation2d(-180)));
        aprilTagMap.put(19, new Pose2d(4.072, 4.761, yawToRotation2d(120)));
        aprilTagMap.put(20, new Pose2d(4.928, 4.806, yawToRotation2d(60)));
        aprilTagMap.put(21, new Pose2d(5.364, 4.010, yawToRotation2d(0)));
        aprilTagMap.put(22, new Pose2d(4.913, 3.304, yawToRotation2d(-60)));
    }

    @Override
    public void periodic() {
        updateRobotPose();
        updateDetectedTagId();
        SmartDashboard.putNumber("Robot ile tag arasındaki açı:", botpose.getDoubleArray(new double[6])[5]);
    }

    public double getTid(){
        return tagIdEntry.getDouble(-1);
    }

    private void updateRobotPose() {
        double[] botPoseArray = botpose.getDoubleArray(new double[6]);
       // System.out.println("Bot pose arrayden gelen deger:"+botPoseArray[0]);
        if (botPoseArray.length == 6) {
            double x = botPoseArray[0];
            double y = botPoseArray[1];
            double theta = botPoseArray[5];

            // Eğer değerler santimetre cinsindeyse metreye çevir
            if (Math.abs(x) > 100 || Math.abs(y) > 100) {
                x /= 100.0;
                y /= 100.0;
            }

            robotPose = new Pose2d(new Translation2d(x, y), new Rotation2d(theta));
            System.out.println("Robot Pose from Limelight: " + robotPose);
        } else {
            
        }
    }

    private void updateDetectedTagId() {
        if (!(tagIdEntry.getNumber(-1).intValue()==-1)) {
            detectedTagId = tagIdEntry.getNumber(-1).intValue();
            System.out.println("Detected Tag ID: " + detectedTagId);
        } else {
            
            detectedTagId = -1;
        }
    }

    public double getTheDegreeBetweenRobotAndApril(){
        return botpose.getDoubleArray(new double[6])[5];
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    public boolean anyAprilTagDetected(){
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(mt2.tagCount==0){
            return false;
        }
        return true;
    }

    public Map<Integer, Pose2d> getAprilTagPoses(){
        return aprilTagMap;
    }

    public Pose2d getAprilTagPose(int tagId) {
        return aprilTagMap.getOrDefault(tagId, null);
    }

    private Rotation2d yawToRotation2d(double yawDegrees) {
        return new Rotation2d(Math.toRadians(yawDegrees));
    }
}