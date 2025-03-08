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

public class IntakeSubsystem extends SubsystemBase {
  SparkMax rotSparkMax;
  SparkMax powSparkMax;
  RelativeEncoder rotRelativeEncoder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
    rotSparkMax = new SparkMax(Constants.IntakeConstants.INTAKE_ROT_SPARKMAX_ID, MotorType.kBrushless);
    powSparkMax = new SparkMax(Constants.IntakeConstants.INTAKE_POW_SPARKMAX_ID, MotorType.kBrushless);  
    rotRelativeEncoder = rotSparkMax.getEncoder();

    // Set encoder value to start pose
   // rotRelativeEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current intake degree position", getRotationDegree());
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
