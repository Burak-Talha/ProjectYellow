// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CleanerSubsystem extends SubsystemBase {

  SparkMax cleanerRot = new SparkMax(Constants.CleanerConstants.CLEANER_ROT_SPARKMAX_ID, MotorType.kBrushless);
  private RelativeEncoder rotRelativeEncoder;

  private PIDController rotController = new PIDController(Constants.CleanerConstants.KP, Constants.CleanerConstants.KI, Constants.CleanerConstants.KD);

  private TrapezoidProfile.Constraints profiledFeedbackConstraints = 
  new TrapezoidProfile.Constraints(Constants.CleanerConstants.MAX_VELOCITY,
                                   Constants.CleanerConstants.MAX_ACCELERATION);

  private ProfiledPIDController profiledRotController = new ProfiledPIDController(Constants.CleanerConstants.KP,
                                                                                  Constants.CleanerConstants.KI,
                                                                                  Constants.CleanerConstants.KD,
                                                                                  profiledFeedbackConstraints);

  private ArmFeedforward armFeedforward = new ArmFeedforward(Constants.CleanerConstants.KS,
                                                             Constants.CleanerConstants.KG,
                                                             Constants.CleanerConstants.KV,
                                                             Constants.CleanerConstants.KA);


  public enum DesiredCleanerPosition{
    LOWER_ALGAE, UPPER_ALGAE, DEFAULT
  }

  double armDegreeSetpoint = Constants.CleanerConstants.DEFAULT_CLEANER_DEGREE;

  /** Creates a new Cleaner. */
  public CleanerSubsystem() {
    rotRelativeEncoder = cleanerRot.getEncoder();
    rotRelativeEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Get rotation degree :", getRotationDegree());
    //calculateFeedbackDemand();
  }

  public void setDesiredPosition(DesiredCleanerPosition desiredCleanerPosition){
    if(desiredCleanerPosition==DesiredCleanerPosition.LOWER_ALGAE){
      armDegreeSetpoint = Constants.CleanerConstants.LOWER_ALGAE_DEGREE;
    }else if(desiredCleanerPosition==DesiredCleanerPosition.UPPER_ALGAE){
      armDegreeSetpoint = Constants.CleanerConstants.UPPER_ALGAE_DEGREE;
    }else if(desiredCleanerPosition==DesiredCleanerPosition.DEFAULT){
      armDegreeSetpoint = Constants.CleanerConstants.DEFAULT_CLEANER_DEGREE;
    }
  }

  public void calculateFeedbackDemand(){
    double demand = rotController.calculate(getRotationDegree(), armDegreeSetpoint);
    applyDemand(demand);
  }

  public void calculateProfiledDemand(DesiredCleanerPosition desiredCleanerPosition){
    if(desiredCleanerPosition==DesiredCleanerPosition.LOWER_ALGAE){
      armDegreeSetpoint = Constants.CleanerConstants.LOWER_ALGAE_DEGREE;
    }else if(desiredCleanerPosition==DesiredCleanerPosition.UPPER_ALGAE){
      armDegreeSetpoint = Constants.CleanerConstants.UPPER_ALGAE_DEGREE;
    }else if(desiredCleanerPosition==DesiredCleanerPosition.DEFAULT){
      armDegreeSetpoint = Constants.CleanerConstants.DEFAULT_CLEANER_DEGREE;
    }
    double demand = profiledRotController.calculate(getRotationDegree(), armDegreeSetpoint);
    applyDemand(demand);
  }

  public void calculateProfiledFFDemand(){
    double demand = profiledRotController.calculate(getRotationDegree(), armDegreeSetpoint) + armFeedforward.calculate(Math.toRadians(getRotationDegree()), profiledRotController.getSetpoint().velocity);
    applyDemand(demand);
  }

  public void applyDemand(double demand){
    cleanerRot.setVoltage(demand);
  }

  public void getIn(){
    cleanerRot.set(0.05);
  }

  public void getOut(){
    cleanerRot.set(-0.05);
  }

  public boolean atSetpoint(){
    return MathUtil.isNear(armDegreeSetpoint, getRotationDegree(), 10);
  }

  public double getRotationDegree(){
    return getPosition()*Constants.CleanerConstants.POSITION_2_DEGREE;
  }

  public double getPosition(){
    return rotRelativeEncoder.getPosition();
  }

  public void stopMotors(){
    stopRotMotor();
  }

  public void stopRotMotor(){
    cleanerRot.stopMotor();
  }
}
