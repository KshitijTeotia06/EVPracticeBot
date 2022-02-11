// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeBall extends CommandBase {
  /** Creates a new IntakeBall. */
  Intake intake;
  // Shooter shoot;
  Joystick j1;
  boolean on = false;

  public IntakeBall(Intake intake, Joystick j1) {
    addRequirements(intake);
    this.intake = intake;
    //this.shoot = shoot;
    this.j1 = j1;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("7 button", j1.getRawButton(7));
    if(j1.getTrigger()){
      intake.intakeBrush(0.8);
      intake.intakeBall(0.8);
      intake.transitionMotor(1);
    }else{
      intake.intakeBrush(0);
      intake.intakeBall(0);
      intake.transitionMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
