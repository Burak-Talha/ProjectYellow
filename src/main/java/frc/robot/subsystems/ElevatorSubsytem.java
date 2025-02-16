// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class ElevatorSubsytem extends SubsystemBase {

  private SparkMax leftMax = new SparkMax(Constants.ElevatorConstants.LEFT_ELEVATOR_ID, MotorType.kBrushless);
  private SparkMax rightMax = new SparkMax(Constants.ElevatorConstants.RIGHT_ELEVATOR_ID, MotorType.kBrushless);
  private PIDController elevatorPidController = new PIDController(Constants.ElevatorConstants.KP, Constants.ElevatorConstants.KI, Constants.ElevatorConstants.KD);
  private RelativeEncoder leftRelativeEncoder;
  private RelativeEncoder rightRelativeEncoder;

  private double setpoint = 0;

  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsytem() {
    leftRelativeEncoder = leftMax.getEncoder();
    rightRelativeEncoder = rightMax.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw left encoder data(position):", leftRelativeEncoder. getPosition());
    SmartDashboard.putNumber("Left encoder data(in meters):", leftRelativeEncoder.getPosition()*Constants.ElevatorConstants.POSITION_2_DISTANCE);
    // Will be added the pid logic
  }

  public void setElevatorSetpoint(double setpoint){
    this.setpoint = setpoint;
  }

  public void elevatorUp(){
    leftMax.set(-0.3);
    rightMax.set(-0.3);
  }

  public void elevatorDown(){
    leftMax.set(0.3);
    rightMax.set(0.3);
  }

  public void stop(){
    leftMax.set(0);
    rightMax.set(0);
  }

}
