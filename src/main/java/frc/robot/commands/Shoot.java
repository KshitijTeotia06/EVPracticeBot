// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;


public class Shoot extends CommandBase {
  private Shooter shoot;
  private Intake intake;
  private Joystick stick;
  private double throttle;
  private boolean shooterWarmedUp = false;

  /** Creates a new Shoot. */
  public Shoot(Shooter shoot, Joystick stick, Intake intake) {
    addRequirements(shoot);
    this.shoot = shoot;
    this.stick = stick;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stick.getRawButton(4)) { // starts warming up shooter (press again to stop shooter)
      if(shooterWarmedUp == false) {
        shooterWarmedUp = true;
      } else {
        shooterWarmedUp = false;
      }
    }

    if(shooterWarmedUp = true){
      shoot.outtakeBall(0.5); // keeps shooter spinning
    }

    if(stick.getTrigger()){ // pulls ball into shooter and speeds up shooter
      shoot.outtakeBall(0.8); // SPEED SHOULD BE DECIDED BY LIMELIGHT AUTO TRAJECTORY IN FINAL BOT
      intake.transitionMotor(0.8);
    }
    SmartDashboard.putNumber("SHOOTER SPEED", shoot.getRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.outtakeBall(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
