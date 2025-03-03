// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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

  private double currentElevatorHeight = 0;
  private double elevatorHeightSetpoint = Constants.ElevatorConstants.L1_ELEVATOR_HEIGHT;

  public enum DesiredElevatorPosition{
    L4, L3, L2, L1
  }

  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsytem() {
    rightRelativeEncoder = rightMax.getEncoder();
    rightRelativeEncoder.setPosition(0);
    currentElevatorHeight = Constants.ElevatorConstants.L1_ELEVATOR_HEIGHT;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw right encoder data(position):", rightRelativeEncoder. getPosition());
    SmartDashboard.putNumber("Right encoder data(in meters):", rightRelativeEncoder.getPosition()*Constants.ElevatorConstants.POSITION_2_DISTANCE);
    
    // Calculate the current elevator height
    currentElevatorHeight = rightRelativeEncoder.getPosition()*Constants.ElevatorConstants.POSITION_2_DISTANCE;
    // Will be added the pid logic
    calculateDemand();
  }

  public void setDesiredElevatorPosition(DesiredElevatorPosition desiredElevatorPosition){
    if(desiredElevatorPosition==DesiredElevatorPosition.L1){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L1_ELEVATOR_HEIGHT;
    }else if(desiredElevatorPosition==DesiredElevatorPosition.L2){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L2_ELEVATOR_HEIGHT;
    }else if(desiredElevatorPosition==DesiredElevatorPosition.L3){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L3_ELEVATOR_HEIGHT;
    }else{
      elevatorHeightSetpoint = Constants.ElevatorConstants.L4_ELEVATOR_HEIGHT;
    }
  }

  public void calculateDemand(){
    double demand = elevatorPidController.calculate(getRightDistance(), elevatorHeightSetpoint);
    leftMax.set(-demand);
    rightMax.set(demand);
  }

  public void elevatorUp(){
    leftMax.set(0.3);
    rightMax.set(-0.3);
  }

  public void elevatorDown(){
    leftMax.set(-0.3);
    rightMax.set(0.3);
  }

  public void stopMotors(){
    leftMax.set(0);
    rightMax.set(0);
  }

  public boolean atSetpoint(){
    return MathUtil.isNear(elevatorHeightSetpoint, currentElevatorHeight, 0.05);
  }

  public double getRightDistance(){
    return rightRelativeEncoder.getPosition()*Constants.ElevatorConstants.POSITION_2_DISTANCE;
  }

  public double getLeftPosition(){
    return leftRelativeEncoder.getPosition();
  }

}
