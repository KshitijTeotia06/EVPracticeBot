// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase {
  private Intake intake;
  private Joystick stick;
  private XboxController xbox;
  private double throttle;
  private double thresholdpos = 0.1;
  private double thresholdneg = -0.1;

  /** Creates a new IntakeBall. */
  public IntakeBall(Intake intake, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.xbox = xbox;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    throttle = stick.getRawAxis(5) - stick.getRawAxis(4);
    if ((thresholdpos >= throttle) || (thresholdneg <= throttle)){
      intake.intakeBall(0);
    } else {
      intake.intakeBall(throttle);
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
