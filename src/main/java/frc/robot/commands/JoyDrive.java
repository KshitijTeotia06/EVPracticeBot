// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class JoyDrive extends CommandBase {
  /** Creates a new JoyDrive. */
  private final Drivetrain drivetrain;
  //private Joystick driveStick, turnStick;
  private XboxController controller;
  public JoyDrive(Drivetrain dt, XboxController xbox) { //replace parameters w (Drivetrain dt, Joystick dst, Joystick tst) for wheel and stick 
    drivetrain = dt;      
    /*driveStick = dst;
    turnStick = tst; */ 
    this.controller = xbox;
    addRequirements(dt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //driveStick = new Joystick(Constants.DRIVE_STICK_PORT);
    //turnStick = new Joystick(Constants.TURN_STICK_PORT);
    controller = new XboxController(Constants.XBOX_PORT);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.move(controller.getRawAxis(Constants.DRIVE_AXIS), controller.getRawAxis(Constants.TURN_AXIS)); 
    /* replace parameters w 
    driveStick.getRawAxis(Constants.DRIVE_AXIS), turnStick.getRawAxis(Constants.TURN_STICK_PORT) 
    for final bot with joystick and wheel */
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
