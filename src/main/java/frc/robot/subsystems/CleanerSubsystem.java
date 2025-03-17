// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CleanerSubsystem extends SubsystemBase {

  SparkMax cleanerRot = new SparkMax(Constants.CleanerConstants.CLEANER_ROT_SPARKMAX_ID, MotorType.kBrushless);

  private RelativeEncoder rotRelativeEncoder;
  private PIDController rotController = new PIDController(getPosition(), getRotationDegree(), getPosition());
  private ProfiledPIDController profiledRotController = new ProfiledPIDController(Constants.CleanerConstants.KP, getRotationDegree(), getPosition(), null);
  private ArmFeedforward armFeedforward = new ArmFeedforward(Constants.CleanerConstants.KS, Constants.CleanerConstants.KG, Constants.CleanerConstants.KV, Constants.CleanerConstants.KA);


  /** Creates a new Cleaner. */
  public CleanerSubsystem() {
    rotRelativeEncoder = cleanerRot.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Get rotation degree :", getRotationDegree());

  }

  public void getIn(){
    cleanerRot.set(0.15);
  }

  public void getOut(){
    cleanerRot.set(-0.15);
  }

  public void calculateFeedBackDemand(){

  }

  public void applyDemand(double demand){
    cleanerRot.setVoltage(demand);
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
