// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class JoyDrive extends CommandBase {
  /** Creates a new JoyDrive. */
  private final Drivetrain drivetrain;
  private Joystick driveStick, turnStick;
  private AHRS ahrsNavX;
  private boolean highGear = false;

  // Auto
  private final IntakeBall intake;


  public JoyDrive(Drivetrain dt, Joystick dst, Joystick tst, IntakeBall intake) { //replace parameters w (Drivetrain dt, Joystick dst, Joystick tst) for wheel and stick 
    this.drivetrain = dt;      
    this.turnStick = tst; 
    this.driveStick = dst;
    this.intake = intake;
    
    // Change based on the connection to nav x
    /*
    ahrsNavX = new AHRS(SerialPort.Port.kUSB);
    ahrsNavX.reset();
    ahrsNavX.resetDisplacement();
    ahrsNavX.calibrate();
    */

    addRequirements(dt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveStick = new Joystick(Constants.DRIVE_STICK_PORT);
    turnStick = new Joystick(Constants.TURN_STICK_PORT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putData("shift gear", drivetrain.toggleGear());

    if(driveStick.getTriggerPressed()){
      SmartDashboard.putBoolean("CLICKED", true);
      SmartDashboard.updateValues();
      if(highGear){
        drivetrain.setForward();
      }else{
        drivetrain.setReverse();
      }
      highGear = !highGear;
    }

    drivetrain.move(driveStick.getY(), turnStick.getX());
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
