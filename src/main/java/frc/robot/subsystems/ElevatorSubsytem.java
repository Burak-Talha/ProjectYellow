// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsytem extends SubsystemBase {

  private double currentElevatorHeight = 0;
  private double elevatorHeightSetpoint = Constants.ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT;

  public enum DesiredElevatorPosition{
    L4, L3, L2, L1, UPPER_ALGAE_CLEAN, LOWER_ALGAE_CLEAN, DEFAULT
  }

  public TalonFX elevatorTalon = new TalonFX(Constants.ElevatorConstants.ELEVATOR_ID);
  private PositionVoltage positionRequest;
  private VoltageOut voltageOut = new VoltageOut(0);
  TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();

  public boolean attemptingZeroing = false;
  public boolean hasZeroed = false;
  public boolean atSetpoint = false;

  public DesiredElevatorPosition desiredElevatorPosition = DesiredElevatorPosition.DEFAULT;

  MotionMagicVoltage motionRequest;

  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsytem() {
    configureMotor();
    motionRequest = new MotionMagicVoltage(0);
  }

  @Override
  public void periodic() {
    applyDemand();
    SmartDashboard.putNumber("DEBUG ELEVATOR HEIGHT", DEBUGELEVATORHEIGHT());
    SmartDashboard.putNumber("CURRENT DEBUG ELEVATOR HEIGHT", CURRENTDEBUGELEVATORHEIGHT());
    SmartDashboard.putBoolean("AT SETPOINT", (DEBUGELEVATORHEIGHT()-CURRENTDEBUGELEVATORHEIGHT() < 1.2));
    atSetpoint = (DEBUGELEVATORHEIGHT()-CURRENTDEBUGELEVATORHEIGHT() < 1.7);

  }

  public void setDesiredElevatorPosition(DesiredElevatorPosition desiredElevatorPosition){
    if(desiredElevatorPosition==DesiredElevatorPosition.L1){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L1_ELEVATOR_HEIGHT;
      desiredElevatorPosition = DesiredElevatorPosition.L1;
    }else if(desiredElevatorPosition==DesiredElevatorPosition.L2){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L2_ELEVATOR_HEIGHT;
      desiredElevatorPosition = DesiredElevatorPosition.L2;
    }else if(desiredElevatorPosition==DesiredElevatorPosition.L3){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L3_ELEVATOR_HEIGHT;
      desiredElevatorPosition = DesiredElevatorPosition.L3;
    }else if(desiredElevatorPosition==DesiredElevatorPosition.L4){
      elevatorHeightSetpoint = Constants.ElevatorConstants.L4_ELEVATOR_HEIGHT;
      desiredElevatorPosition = DesiredElevatorPosition.L4;
    }else if(desiredElevatorPosition==DesiredElevatorPosition.LOWER_ALGAE_CLEAN){
      elevatorHeightSetpoint=Constants.ElevatorConstants.LOWER_ALGAE_CLEAN_HEIGHT;
    } else if(desiredElevatorPosition==DesiredElevatorPosition.UPPER_ALGAE_CLEAN){
      elevatorHeightSetpoint=Constants.ElevatorConstants.UPPER_ALGAE_CLEAN_HEIGHT;
    } else if(desiredElevatorPosition==DesiredElevatorPosition.DEFAULT){
      elevatorHeightSetpoint=Constants.ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT;
    }
  }

  public void applyDemand(){
    elevatorTalon.setControl(motionRequest.withPosition(elevatorHeightSetpoint));
  }

  public void elevateUp(){
    elevatorTalon.set(0.3);
  }

  public void elevateDown(){
    elevatorTalon.set(-0.3);
  }

  public void stopMotor(){
    elevatorTalon.stopMotor();
  }

  public boolean atSetpoint(){
    return atSetpoint;
  }

  public double DEBUGELEVATORHEIGHT(){
    return elevatorHeightSetpoint;
  }

  public double CURRENTDEBUGELEVATORHEIGHT(){
    return elevatorTalon.getPosition().getValueAsDouble();
  }

  public DesiredElevatorPosition getDesiredElevatorPosition(){
    return desiredElevatorPosition;
  }

  public void configureMotor(){

      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      // Elevator motors will provide feedback in INCHES the carriage has moved
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.876;

      ELEVATOR_CONFIG.Slot0.kG = 0.3; // Volts to overcome gravity
      ELEVATOR_CONFIG.Slot0.kS = 0.05; // Volts to overcome static friction
      ELEVATOR_CONFIG.Slot0.kV = 0.0005; // Volts for a velocity target of 1 rps
      ELEVATOR_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
      ELEVATOR_CONFIG.Slot0.kP = 0.4;
      ELEVATOR_CONFIG.Slot0.kI = 0.0;
      ELEVATOR_CONFIG.Slot0.kD = 0.001;
      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;

      ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 400;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 800;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicExpo_kV = 0.12;

      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = false;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 100;
      ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = false;
      ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 100;
      elevatorTalon.getConfigurator().apply(ELEVATOR_CONFIG);
  }

}
