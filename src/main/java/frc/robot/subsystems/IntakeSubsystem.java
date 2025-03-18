// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  SparkMax rotSparkMax = new SparkMax(Constants.IntakeConstants.INTAKE_ROT_SPARKMAX_ID, MotorType.kBrushless);
  SparkMax powSparkMax = new SparkMax(Constants.IntakeConstants.INTAKE_POW_SPARKMAX_ID, MotorType.kBrushless);

  RelativeEncoder rotRelativeEncoder;

  // Define the controllers to control mechanism
  private PIDController intakeRotPidController =
  new PIDController(Constants.CleanerConstants.KP,
                    Constants.CleanerConstants.KI,
                    Constants.CleanerConstants.KD);

  private ElevatorFeedforward intakeRotFeedforwardController =
  new ElevatorFeedforward(Constants.CleanerConstants.KS,
                          Constants.CleanerConstants.KG,
                          Constants.CleanerConstants.KV,
                          Constants.CleanerConstants.KA);

  private TrapezoidProfile.Constraints profiledFeedbackConstraints = 
  new TrapezoidProfile.Constraints(Constants.CleanerConstants.MAX_VELOCITY,
                                   Constants.CleanerConstants.MAX_ACCELERATION);

  private ProfiledPIDController profiledIntakeRotPidController =
  new ProfiledPIDController(Constants.CleanerConstants.KP,
                            Constants.CleanerConstants.KI,
                            Constants.CleanerConstants.KD,
                            profiledFeedbackConstraints);

  private double currentIntakeDegree = 0;
  private double intakeDegreeSetpoint = Constants.CleanerConstants.DEFAULT_CLEANER_DEGREE;

  public enum DesiredIntakePosition{
    INTAKE, DEFAULT
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    rotRelativeEncoder = rotSparkMax.getEncoder();
    // Set encoder value to start pose
    rotRelativeEncoder.setPosition(Constants.IntakeConstants.DEFAULT_INTAKE_DEGREE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current intake degree position", getRotationDegree());
    // Check it out!
    //calculateFeedBackDemand();
    //calculateProfiledDemand();
    //calculateProfiledFFDemand();
  }

  public void setDesiredIntakePosition(DesiredIntakePosition desiredIntakePosition){
    if(desiredIntakePosition==DesiredIntakePosition.DEFAULT){
      currentIntakeDegree = Constants.IntakeConstants.DEFAULT_INTAKE_DEGREE;
    } else if(desiredIntakePosition==DesiredIntakePosition.INTAKE){
      currentIntakeDegree = Constants.IntakeConstants.INTAKE_ALGAE_DEGREE;
    }
  }

  public void calculateFeedBackDemand(){
    double demand = intakeRotPidController.calculate(getRotationDegree(), intakeDegreeSetpoint);
    applyDemand(demand);
  }

  public void calculateProfiledDemand(){
    double demand = profiledIntakeRotPidController.calculate(getRotationDegree(), intakeDegreeSetpoint);
    applyDemand(demand);
  }

  public void calculateProfiledFFDemand(){
    double demand = profiledIntakeRotPidController.calculate(getRotationDegree(), intakeDegreeSetpoint) +
                    intakeRotFeedforwardController.calculate(profiledIntakeRotPidController.getSetpoint().velocity);
    applyDemand(demand);
  }

  public void applyDemand(double demand){
    rotSparkMax.setVoltage(demand);
  }

  public void intakeUp(){
    rotSparkMax.set(0.3);
  }

  public void intakeDown(){
    rotSparkMax.set(-0.3);
  }

  public void intake(){
    powSparkMax.set(0.5);
  }

  public void outtake(){
    powSparkMax.set(-0.5);
  }

  public void stopPowerMotor(){
    powSparkMax.stopMotor();
  }

  public void stopRotationMotor(){
    rotSparkMax.stopMotor();
  }

  public double getRotationDegree(){
    return getEncoderPosition()*Constants.IntakeConstants.POSITION_2_DEGREE;
  }

  private double getEncoderPosition(){
    return rotRelativeEncoder.getPosition();
  }
}
