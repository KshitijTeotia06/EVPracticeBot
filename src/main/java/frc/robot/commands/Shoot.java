// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class Shoot extends CommandBase {
  private Shooter shoot;
  private Joystick stick;
  private double throttle;

  /** Creates a new Shoot. */
  public Shoot(Shooter shoot, Joystick stick) {
    addRequirements(shoot);
    this.shoot = shoot;
    this.stick = stick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    throttle = stick.getRawAxis(3) - stick.getRawAxis(2);
    shoot.outtakeBall(throttle);
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
