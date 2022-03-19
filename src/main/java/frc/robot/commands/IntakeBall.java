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
  Intake intake;
  XboxController controller;
  boolean on = false;
  double speed = 0;
  boolean isEjecting = false;
  

  public IntakeBall(Intake intake, XboxController controller) {
    this.intake = intake;
    addRequirements(intake);
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Test_Banner1 ", intake.banner1Output());
    SmartDashboard.putBoolean("TEST_Banner2", intake.banner2Output());
    SmartDashboard.updateValues();
    
    intake.mainIntakeFunction(controller.getRightY());
    // intake.conveyor(controller.getRightY());


    if (controller.getRawButtonPressed(4)){
      intake.intakeToggle();
    }
    // intake.intake(isEjecting);
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