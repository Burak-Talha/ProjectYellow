// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

      public Pigeon2 gyro;
      public TalonFX First_Elevator_Kraken;
      public TalonFX Second_Elevator_Kraken;
      public CANcoder Elevator_Cancoder;
    
      public PIDController elevator_PidController;

  /** Creates a new ElevatorSubsytem. */
  public Elevator() {

    First_Elevator_Kraken = new TalonFX(Constants.First_ElevatorID, "Canivore");
    Second_Elevator_Kraken = new TalonFX(Constants.Second_ElevatorID, "Canivore");
    Elevator_Cancoder = new CANcoder(Constants.Elevator_CancoderID, "Canivore");

    elevator_PidController = new PIDController(0, 0, 0);

    StatusSignal<Angle> Elevator_AbsultePosition = Elevator_Cancoder.getAbsolutePosition();

  }

  @Override
  public void periodic() {



    // This method will be called once per scheduler run
  }

  public void Update_Elevator_Status() {



  }

}
