// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cleaner extends SubsystemBase {

  SparkMax cleaner = new SparkMax(35, MotorType.kBrushless);

  /** Creates a new Cleaner. */
  public Cleaner() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getIn(){
    cleaner.set(0.5);
  }

  public void getOut(){
    cleaner.set(-0.5);
  }

  public void stopMotors(){
    cleaner.set(0);
  }
}
