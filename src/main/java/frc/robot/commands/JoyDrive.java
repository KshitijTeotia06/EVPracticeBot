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
  //private Joystick driveStick, turnStick;
  private XboxController controller;
  private AHRS ahrsNavX;

  public JoyDrive(Drivetrain dt, XboxController xbox) { //replace parameters w (Drivetrain dt, Joystick dst, Joystick tst) for wheel and stick 
    drivetrain = dt;      
    /*driveStick = dst;
    turnStick = tst; */ 
    this.controller = xbox;
     
    // Change based on the connection to nav x
  
    ahrsNavX = new AHRS(SerialPort.Port.kUSB);
    ahrsNavX.reset();
    ahrsNavX.resetDisplacement();
    ahrsNavX.calibrate();
    

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
<<<<<<< HEAD
    if(driveStick.getRawButton(0)){
      drivetrain.toggleGear();
    }
    drivetrain.move(-driveStick.getRawAxis(Constants.DRIVE_AXIS), -turnStick.getX());
=======
    drivetrain.move(controller.getRawAxis(Constants.DRIVE_AXIS), controller.getRawAxis(Constants.TURN_AXIS)); 
    /* replace parameters w 
    driveStick.getRawAxis(Constants.DRIVE_AXIS), turnStick.getRawAxis(Constants.TURN_STICK_PORT) 
    for final bot with joystick and wheel */
    
    SmartDashboard.putNumber("gyroAngleX", ahrsNavX.getAngle());
    SmartDashboard.updateValues();

>>>>>>> d13f6b3d4862082d3b5dc7ba01f755ddcb627aa5
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
