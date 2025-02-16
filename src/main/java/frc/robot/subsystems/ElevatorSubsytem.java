// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private DesiredElevatorPosition currentDesiredElevatorPosition;
  private double elevatorHeightSetpoint = Constants.ElevatorConstants.DEFAULT_ELEVATOR_HEIGHT;

  public enum DesiredElevatorPosition{
    L4, L3, L2, DEFAULT_ELEVATOR_HEIGHT
  }

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
    //runMotors(calculateDemand());
  }

  public double calculateDemand(){
    return elevatorPidController.calculate(getLeftPosition(), elevatorHeightSetpoint);
  }

  public void setDesiredElevatorPosition(DesiredElevatorPosition desiredElevatorPosition){
    this.currentDesiredElevatorPosition = desiredElevatorPosition;

    if(currentDesiredElevatorPosition==DesiredElevatorPosition.DEFAULT_ELEVATOR_HEIGHT){
      elevatorHeightSetpoint = Constants.ElevatorConstants.DEFAULT_ELEVATOR_HEIGHT;
    }else if(currentDesiredElevatorPosition==DesiredElevatorPosition.L2){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L2_ELEVATOR_HEIGHT;
    }else if(currentDesiredElevatorPosition==DesiredElevatorPosition.L3){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L3_ELEVATOR_HEIGHT;
    }else{
      elevatorHeightSetpoint = Constants.ElevatorConstants.L4_ELEVATOR_HEIGHT;
    }
  }

  public void runMotors(double demand){
    leftMax.set(demand);
    rightMax.set(demand);
  }

  public void elevatorUp(){
    leftMax.set(-0.3);
    rightMax.set(-0.3);
  }

  public void elevatorDown(){
    leftMax.set(0.3);
    rightMax.set(0.3);
  }

  public void stopMotors(){
    leftMax.set(0);
    rightMax.set(0);
  }

  public boolean atSetpoint(){
    return elevatorPidController.atSetpoint();
  }

  public double getLeftDistance(){
    return leftRelativeEncoder.getPosition()*Constants.ElevatorConstants.POSITION_2_DISTANCE;
  }

  public double getLeftPosition(){
    return leftRelativeEncoder.getPosition();
  }

}
