// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  
  CANSparkMax climb1, climb2;

  public Climb() {
    climb1 = new CANSparkMax(Constants.CLIMB_MOTOR_1_ID, MotorType.kBrushless);
    climb2 = new CANSparkMax(Constants.CLIMB_MOTOR_2_ID, MotorType.kBrushless);
    climb2.setInverted(true);
    climb2.follow(climb1);
  }

  public void move(double power){
    climb1.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
