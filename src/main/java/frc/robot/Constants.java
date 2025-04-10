package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.swerve.COTSTalonFXSwerveConstants;
import frc.robot.lib.util.swerve.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    //public static  String kCANbus = "Canivore";


    public static final class Swerve {
   
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        // Distance between right and left wheels
        public static final double trackWidth = Units.inchesToMeters(19.68); //TODO: This must be tuned to specific robot
        // Distance between front and back wheels
        public static final double wheelBase = Units.inchesToMeters(19.68); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 40;
        public static final int angleCurrentThreshold = 60;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;  ///0.25
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1; //TODO: This must be tuned to specific robot  0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static double currentSpeed = 1.5; //TODO: This must be tuned to specific robot
        public static double OTO_currentSpeed = 4.5; //TODO: This must be tuned to specific robot
        public static double defaulSpeed = 1.5;
        public static double l4Speed = 5;
        public static double l3Speed = 70;

        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(147);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 23;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 26;
            public static final int angleMotorID = 27;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-110);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 24;
            public static final int angleMotorID = 25;
            public static final int canCoderID = 30;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(150);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AlignmentConstants{
        public static final double DISTANCE_ALIGMENT_TOLERANCE = 0.1;
        public static final double ANGLE_ALIGMENT_TOLERANCE = 5;

        public static final double X_CONTROLLER_P = 1.2;
        public static final double X_CONTROLLER_I = 0.001;
        public static final double X_CONTROLLER_D = 0;

        public static final double Y_CONTROLLER_P = 1.2;
        public static final double Y_CONTROLLER_I = 0.001;
        public static final double Y_CONTROLLER_D = 0;

        public static final double Z_CONTROLLER_P = 0.08;
        public static final double Z_CONTROLLER_I = 0.001;
        public static final double Z_CONTROLLER_D = 0;
    }
 
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 5;
        public static final double kPYController = 5;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ElevatorConstants{
        public static final int ELEVATOR_ID = 36;
        public static final int ELEVATOR_CANCODER_ID = 10;

        public static final double KP = 0.05;
        public static final double KI = 0;
        public static final double KD = 0.0001;

        public static final double KS = 0;
        public static final double KV = 0.009;
        public static final double KG = 0.5;
        public static final double KA = 0.000;
        
        private static final double GEAR_BOX_RATIO = 10.71;
        private static final double GEAR_RADIUS = 2.5;
        private static final double GEAR_CIRCUMFERENCE = 2*Math.PI*GEAR_RADIUS;
        public static final double POSITION_2_DISTANCE = (1/GEAR_BOX_RATIO)*GEAR_CIRCUMFERENCE;
        
        public static final double ELEVATOR_DEFAULT_HEIGHT = 0;
        public static final double L4_ELEVATOR_HEIGHT = 80.5;
        public static final double L3_ELEVATOR_HEIGHT = 40;
        public static final double L2_ELEVATOR_HEIGHT = 17;
        public static final double L1_ELEVATOR_HEIGHT = 0;
        public static final double LOWER_ALGAE_CLEAN_HEIGHT = 30;
        public static final double UPPER_ALGAE_CLEAN_HEIGHT = 25;

        public static final double MAX_VELOCITY = 5;
        public static final double MAX_ACCELERATION = 5;
    }

    public static final class GripperConstants{
        public static final int LEFT_SPARKMAX_ID = 10;
        public static final int RIGHT_SPARKMAX_ID = 18;
        public static final int FRONT_INFRARED_SENSOR_ID = 1;
        public static final int REAR_INFRARED_SENSOR_ID = 0;
    }

    public static final class CleanerConstants{
        public static final int CLEANER_ROT_SPARKMAX_ID = 3;
        public static final int CLEANER_POW_SPARKMAX_ID = 35;

        public static final double GEARBOX_RATIO = 40;
        public static final double PULLEY_RATIO = 1;// This will be known
        public static final double POSITION_2_DEGREE = 360/(GEARBOX_RATIO*PULLEY_RATIO);

        public static final double KP = 0.3;
        public static final double KI = 0;
        public static final double KD = 0;
        
        public static final double KG = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KS = 0;

        public static final double MAX_VELOCITY = 250;
        public static final double MAX_ACCELERATION = 175;

        public static final double DEFAULT_CLEANER_DEGREE = -290;
        public static final double LOWER_ALGAE_DEGREE = 0;
        public static final double UPPER_ALGAE_DEGREE = -140;
    }
    
    public static final class IntakeConstants{
        public static final int INTAKE_ROT_SPARKMAX_ID = 37;
        public static final int INTAKE_POW_SPARKMAX_ID = 33;

        public static final double GEARBOX_RATIO = 48;
        public static final double POSITION_2_DEGREE = 360/GEARBOX_RATIO;

        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        
        public static final double KG = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KS = 0;

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final double DEFAULT_INTAKE_DEGREE = 0;
        public static final double INTAKE_ALGAE_DEGREE = 60;

    }
}
