// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

  private SparkMax leftSparkMax = new SparkMax(Constants.GripperConstants.LEFT_SPARKMAX_ID, MotorType.kBrushless);
  private SparkMax rightSparkMax = new SparkMax(Constants.GripperConstants.RIGHT_SPARKMAX_ID, MotorType.kBrushless);

  private DigitalInput frontInfrared = new DigitalInput(Constants.GripperConstants.FRONT_INFRARED_SENSOR_ID);
  private DigitalInput rearInfrared = new DigitalInput(Constants.GripperConstants.REAR_INFRARED_SENSOR_ID);

  private SparkBaseConfig config = new SparkMaxConfig();
  private ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

  /** Creates a new CoralintakeSubsystem. */
  public GripperSubsystem() {
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Has Coral?", hasCoral());
    SmartDashboard.putBoolean("FRONT INFRARED BOOL : ", frontInfrared.get());
    SmartDashboard.putBoolean("REAR INFRARED BOOL : ", rearInfrared.get());

    closedLoopConfig.p(0.5, ClosedLoopSlot.kSlot0);
    config.apply(closedLoopConfig);
    
    leftSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void fasterGetIn(){
    leftSparkMax.getClosedLoopController().setReference(0.3, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    rightSparkMax.getClosedLoopController().setReference(-0.3, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  public void slowerGetIn(){
    leftSparkMax.getClosedLoopController().setReference(0.17, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    rightSparkMax.getClosedLoopController().setReference(-0.17, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  public void getOut(){
    leftSparkMax.getClosedLoopController().setReference(0.5, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    rightSparkMax.getClosedLoopController().setReference(-0.5, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  public void getOutSlower(){
    leftSparkMax.getClosedLoopController().setReference(0.3, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
    rightSparkMax.getClosedLoopController().setReference(-0.3, ControlType.kDutyCycle, ClosedLoopSlot.kSlot0);
  }

  public void stopMotors(){
    leftSparkMax.set(0);
    rightSparkMax.set(0);
  }

  public boolean hasCoral(){
    return (frontInfrared.get() && !rearInfrared.get());
  }

  public boolean isCoralEjected(){
    return frontInfrared.get();
  }

  public boolean frontInfrared(){
    return frontInfrared.get();
  }

  public boolean rearInfrared(){
    return rearInfrared.get();
  }
}
