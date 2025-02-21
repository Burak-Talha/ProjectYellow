// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

  private SparkMax leftSparkMax = new SparkMax(Constants.GripperConstants.LEFT_SPARKMAX_ID, MotorType.kBrushless);
  private SparkMax rightSparkMax = new SparkMax(Constants.GripperConstants.RIGHT_SPARKMAX_ID, MotorType.kBrushless);

  /** Creates a new CoralintakeSubsystem. */
  public GripperSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getIn(){
    leftSparkMax.set(-0.3);
    rightSparkMax.set(0.3);
  }

  public void getOut(){
    leftSparkMax.set(0.3);
    rightSparkMax.set(-0.3);
  }

  public void stopMotors(){
    leftSparkMax.set(0);
    rightSparkMax.set(0);
  }
}
