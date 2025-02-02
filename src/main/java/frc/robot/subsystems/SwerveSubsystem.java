package frc.robot.subsystems;

import java.io.IOException;
import java.util.Arrays;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.lib.util.TargetReef;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    RobotConfig config;
    Field2d field2d = new Field2d();
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    Translation3d robotpose;
    Pose3d pose3d = new Pose3d();

    // Limelight veri alımı için gerekli girişler
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry botpose = limelightTable.getEntry("botpose");

    private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.0000003, 0.000003, Math.toRadians(10));
    private static final Matrix<N3,N1> localMeasurementStdDevs = VecBuilder.fill(0.2, 0.2, Math.toRadians(3));

    public SwerveSubsystem() {  
        //Translation2d setpointTranslation2d = aprilTagFieldLayout.getTagPose(1).get().toPose2d().getTranslation().minus((swerveOdometry.getPoseMeters()));
        //xpidController.setpoint(setpointTranslation2d.getX());
        SmartDashboard.putData(field2d);

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(),
        localMeasurementStdDevs,
        visionMeasurementStdDevs);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                        new PIDConstants(50, 0.0, 0),
                        new PIDConstants(0.1, 0.0, 0.0)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetRobotPoseByLimelight(){
        Pose2d tempPose;
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // Todo işlem kalabalığını azalt.
        tempPose = new Pose2d(mt2.pose.getX(), mt2.pose.getY(), new Rotation2d(3.14+mt2.pose.getRotation().getRadians()));
        swerveDrivePoseEstimator.resetPose(tempPose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

        // Range : 0-360+ - 0-(-360) continous
    public double getGyroAngle(){
        if(gyro.getAngle() < 0){
            double temp_angle = gyro.getAngle() % 360;
            double currentAngle = 360 + temp_angle;
            return currentAngle;
            }
            return gyro.getAngle() % 360;
        }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void addVisionMeasurementByThresholds(PoseEstimate poseEstimate1, PoseEstimate poseEstimate2){
        swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(4,4,99999999));
        try{
        if(poseEstimate1.tagCount!=0){
            swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate1.pose, poseEstimate1.timestampSeconds);
        }
        /*if(poseEstimate2.tagCount!=0){
            swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate2.pose, poseEstimate2.timestampSeconds);
        }*/
    }catch(Exception exception){
        exception.printStackTrace();
    }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        try {
            SmartDashboard.putNumber("Robot kg:", RobotConfig.fromGUISettings().massKG);
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        LimelightHelpers.SetRobotOrientation("limelight", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        LimelightHelpers.PoseEstimate mt2_arka = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-arka");
        addVisionMeasurementByThresholds(mt2, mt2_arka);
        swerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("ROBOT DEGREES", getGyroAngle());
        SmartDashboard.putNumber("ROBOT X", swerveDrivePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("ROBOT Y", swerveDrivePoseEstimator.getEstimatedPosition().getY());

        field2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    }

    public void stop() {
        drive(new Translation2d(0, 0), 0, true, false); // Motorları durdur
    }  

       Pose2d[] currrentHumanPose2ds;
    public Pose2d getHumanPlayerTarget(){
        if(DriverStation.getAlliance().get()==Alliance.Red){
            currrentHumanPose2ds = FieldConstants.redHumanPlayerTargets;
        } else{
            currrentHumanPose2ds = FieldConstants.blueHumanPlayerTargets;
        }
        
        return swerveDrivePoseEstimator.getEstimatedPosition().nearest(Arrays.asList(currrentHumanPose2ds));
    }

    public Pose2d[] getHumanPlayerABTarget(){
        if(DriverStation.getAlliance().get()==Alliance.Red){
            currrentHumanPose2ds = FieldConstants.redHumanPlayerTargets;
        } else{
            currrentHumanPose2ds = FieldConstants.blueHumanPlayerTargets;
        }
        return currrentHumanPose2ds;    
    }

    public TargetReef[] getDeployReefTarget(){
        if(DriverStation.getAlliance().get()==Alliance.Red){
            return FieldConstants.RED_TARGET_REEFS;
        } else{
            return FieldConstants.BLUE_TARGET_REEFS;
        }
    }
    
}
