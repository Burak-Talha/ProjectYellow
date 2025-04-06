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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
import frc.robot.lib.util.swerve.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    RobotConfig config;
    Field2d field2d = new Field2d();
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    Pose2d currentTargetPose;

    public enum DesiredBaseChassisSpeed{
        DEFAULT, L3_SPEED, L4_SPEED
    }
    
    // If u gonna trust your data, decrease the coeffecients. Otherwise increase it.
    private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(2.5, 2.5, 9999999);
    private static final Matrix<N3,N1> localMeasurementStdDevs = VecBuilder.fill(0.2, 0.2, Math.toRadians(5));

    public SwerveSubsystem() {  
        //Translation2d set pointTranslation2d = aprilTagFieldLayout.getTagPose(1).get().toPose2d().getTranslation().minus((swerveOdometry.getPoseMeters()));
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

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          e.printStackTrace();
        }


        AutoBuilder.configure(
                this::getEstimatedPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::oto_driveRobotRelative,
                new PPHolonomicDriveController(
                        new PIDConstants(8, 0.0, 0),
                        new PIDConstants(15, 0.0, 0.0) // 0.1
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


        if(DriverStation.getAlliance().get()==Alliance.Blue){
            resetRobotPoseByLimelight();
        }
    }
    

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.currentSpeed);
        setModuleStates(states);
    }
    public void oto_driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.OTO_currentSpeed);
        oto_setModuleStates(states);
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.currentSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.currentSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void oto_setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.OTO_currentSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setMaxSpeed(DesiredBaseChassisSpeed desiredBaseChassisSpeed){
        if(desiredBaseChassisSpeed==DesiredBaseChassisSpeed.DEFAULT){
            Constants.Swerve.currentSpeed = Constants.Swerve.defaulSpeed;
        }else if(desiredBaseChassisSpeed==DesiredBaseChassisSpeed.L4_SPEED){
            Constants.Swerve.currentSpeed = Constants.Swerve.l4Speed;
        }else if(desiredBaseChassisSpeed==DesiredBaseChassisSpeed.L3_SPEED){
            Constants.Swerve.currentSpeed = Constants.Swerve.l3Speed;
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

    public Pose2d getEstimatedPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
       // swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
       swerveDrivePoseEstimator.resetPose(getPose2dmt2());
    }

    public void resetRobotPoseByLimelight(){
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-first");
        // Todo işlem kalabalığını azalt.
        Pose2d tempPose = new Pose2d(mt2.pose.getX(), mt2.pose.getY(), new Rotation2d(3.14+mt2.pose.getRotation().getRadians()));
        //Pose2d tempPose = getEstimatedPose().transformBy(new Transform2d(new Translation2d(), new Rotation2d(Math.PI)));
        swerveDrivePoseEstimator.resetPose(tempPose);
    }

    public Rotation2d getHeading(){
        return getEstimatedPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getEstimatedPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getEstimatedPose().getTranslation(), new Rotation2d()));
    }

    public void setCurrentTargetPose(Pose2d targetPose){
        this.currentTargetPose = targetPose;
    }

    public boolean robotAtSetpoint(){
        double distanceBetweenTarget = currentTargetPose.getTranslation().getDistance(getEstimatedPose().getTranslation());
        return MathUtil.isNear(0, distanceBetweenTarget, Constants.AlignmentConstants.DISTANCE_ALIGMENT_TOLERANCE);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public Pose2d getPose2dmt2(){
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-first");
        return mt2.pose;
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
        try{
        if(poseEstimate1.tagCount!=0){
            swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate1.pose, poseEstimate1.timestampSeconds);
        }
        if(poseEstimate2.tagCount!=0){
            swerveDrivePoseEstimator.addVisionMeasurement(poseEstimate2.pose, poseEstimate2.timestampSeconds);
        }
    }catch(Exception exception){
        
    }
    }

    @Override
    public void periodic(){
        try {
            LimelightHelpers.SetRobotOrientation("limelight", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation("limelight-first", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2First = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            LimelightHelpers.PoseEstimate mt2Second = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-first");
            swerveDrivePoseEstimator.update(getGyroYaw(), getModulePositions());
            addVisionMeasurementByThresholds(mt2First, mt2Second);
        } catch (Exception e) {
        }

        field2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());

        for (SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod"+ mod.moduleNumber + "Cancoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod"+ mod.moduleNumber + "angle", mod.getPosition().angle.getDegrees());

            

        }




    }

    public void stop() {
        drive(new Translation2d(0, 0), 0, true, false); // Motorları durdur
    }  

       Pose2d[] currrentHumanPose2ds;
   /* public Pose2d getHumanPlayerTarget(){
        if(DriverStation.getAlliance().get()==Alliance.Red){
            currrentHumanPose2ds = FieldConstants.redHumanPlayerTargets;
        } else{
            currrentHumanPose2ds = FieldConstants.blueHumanPlayerTargets;
        }
        
        return swerveDrivePoseEstimator.getEstimatedPosition().nearest(Arrays.asList(currrentHumanPose2ds));
    }*/

   /* public Pose2d[] getHumanPlayerABTarget(){
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
    }*/
    
}
