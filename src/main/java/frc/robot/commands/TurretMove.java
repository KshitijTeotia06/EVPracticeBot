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
  Joystick j1;
  Joystick wheel;

  public TurretMove(Turret turret, Joystick j1, Joystick wheel) {
    this.turret = turret;
    addRequirements(turret);
    this.j1 = j1;
    this.wheel = wheel;
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

    //turret.turnTurret(-0.9);
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
