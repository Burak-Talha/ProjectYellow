// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CleanerSubsystem extends SubsystemBase {

  SparkMax cleanerRot;
  SparkMax cleanerPow;
  RelativeEncoder rotRelativeEncoder;

  /** Creates a new Cleaner. */
  public CleanerSubsystem() {
    cleanerRot = new SparkMax(Constants.CleanerConstants.CLEANER_ROT_SPARKMAX_ID, MotorType.kBrushless);
    cleanerPow = new SparkMax(Constants.CleanerConstants.CLEANER_POW_SPARKMAX_ID, MotorType.kBrushless);
    rotRelativeEncoder = cleanerRot.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Get rotation degree :", getRotationDegree());
  }

  public void getIn(){
    cleanerPow.set(0.5);
  }

  public void getOut(){
    cleanerPow.set(-0.5);
  }

  public double getRotationDegree(){
    return getPosition()*Constants.CleanerConstants.POSITION_2_DEGREE;
  }

  public double getPosition(){
    return rotRelativeEncoder.getPosition();
  }

  public void stopMotors(){
    stopPowMotor();
    stopRotMotor();
  }

  public void stopPowMotor(){
    cleanerPow.stopMotor();
  }

  public void stopRotMotor(){
    cleanerRot.stopMotor();
  }
}
