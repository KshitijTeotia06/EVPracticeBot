// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeBall extends CommandBase {
  /** Creates a new IntakeBall. */
  Intake intake;
  // Shooter shoot;
  XboxController controller;
  Joystick j1;
  boolean on = false;
  double speed = 0;

  public IntakeBall(Intake intake, Joystick j1, XboxController controller) {
    this.intake = intake;
    addRequirements(intake);
    //this.shoot = shoot;
    this.j1 = j1;
    this.controller = controller;
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
    SmartDashboard.putBoolean("banner1: ", intake.banner1Output());
    SmartDashboard.putBoolean("banner2: ", intake.banner2Output());
    /*
    if(j1.getRawButton(5)){ 
      intake.intake(true); // ejects balls (joystick button 6(id 5))
    } else {
      intake.intake(false); // keeps running intake by default
    }
    
    if(j1.getRawButton(2)){ //runs brush (add color sensor stuff here)
      intake.intakeBrush(0.8);
    }else{
      intake.intakeBrush(0);
    } 
    */
    SmartDashboard.putBoolean("BANNNER OUTPUT", intake.banner1Output());
    if(!intake.banner1Output()){
      SmartDashboard.putNumber("ITNAKE SPED ", controller.getRightY());
      intake.intakeBall(controller.getRightY());
    }else{
      intake.intakeBall(0);
    }
    // if(j1.getRawButton(12)){
    //   if(speed == 0) speed = 0.9;
    //   else speed = 0;
    // }
    // intake.intakeBall(speed);
    SmartDashboard.updateValues();
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