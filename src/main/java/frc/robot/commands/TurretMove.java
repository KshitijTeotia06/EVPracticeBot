// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretMove extends CommandBase {
  /** Creates a new TurretMove. */
  Turret turret;
  XboxController controller;
  boolean manual;
  Vision vision;

  public TurretMove(Turret turret, XboxController controller, Vision vision) {
    this.turret = turret;
    addRequirements(turret);
    this.vision = vision;
    this.controller = controller;
    manual = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("TURRETTHROTTLE", turretThrottle);
    // turret.turnTurret(-0.1);
    /*
    if (wheel.getRawButton(4)){
      turret.turnTurret(-0.5); // turns turret left
    } else if (wheel.getRawButton(6)){
      turret.turnTurret(0.5); // turns turret right
    }
    */
    // SmartDashboard.putNumber("AXIS VALUE", controller.getRightTriggerAxis());
    // SmartDashboard.updateValues();
    SmartDashboard.putBoolean("MANUAL ", manual);
    SmartDashboard.updateValues();
    if(controller.getBButtonPressed()){
      manual = !manual;
    }
    if(manual){
      if ((controller.getLeftX() < 0.2) && (controller.getLeftX() > -0.2)){
        turret.setSpeed(0);
      } else {
        turret.setSpeed(-controller.getLeftX());
      }
    }else{
      // if ((controller.getLeftX() < 0.2) && (controller.getLeftX() > -0.2)){
      //   turret.turnTurret(0);
      // } else {
      //   // if (vision.getX() < 1.2) {
      //   //   turret.turnTurret(-controller.getLeftX());
      //   // } else {
      //     turret.turnTurret(controller.getLeftX());
      //   // }
      // }
      turret.turnTurret(1);
    }
    // turret.setSpeed(controller.getRightX());
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
