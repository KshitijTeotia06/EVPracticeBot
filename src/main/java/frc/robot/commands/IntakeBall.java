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
  Joystick stick;
  boolean on = false;
  double speed = 0;
  boolean isEjecting = false;
  

  public IntakeBall(Intake intake, XboxController controller, Joystick stick) {
    this.intake = intake;
    this.stick = stick;
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
    
    SmartDashboard.putNumber("RIGHT Y", controller.getRightY());
    SmartDashboard.updateValues();
    // intake.conveyor(controller.getRightY());

    if (controller.getYButtonPressed()) {
      intake.intakeToggle();
    }

    // if (stick.getRawButton(4)) {
    //   if (intake.banner1Output() && intake.banner2Output()) {
    //     intake.mainIntakeFunction(0);
    //   } else {
    //     intake.mainIntakeFunction(0.8);
    //   }
    // }
    // else {
    //   intake.mainIntakeFunction(0);
    // }
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