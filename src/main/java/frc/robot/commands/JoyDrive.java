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
  private XboxController controller;
  private AHRS ahrsNavX;
  private boolean highGear = false;

  // Auto
  private final IntakeBall intake;


  public JoyDrive(Drivetrain dt, XboxController controller, IntakeBall intake) { //replace parameters w (Drivetrain dt, Joystick dst, Joystick tst) for wheel and stick 
    this.drivetrain = dt;      
    this.controller = controller;
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
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putData("shift gear", drivetrain.toggleGear());

    // SmartDashboard.putNumber("ENCODER", drivetrain.getIntegratedSensor());
    // SmartDashboard.updateValues();
    if(controller.getAButtonPressed()){
      // SmartDashboard.putBoolean("CLICKED", true);
      // SmartDashboard.updateValues();
      if(highGear){
        drivetrain.setForward();
      }else{
        drivetrain.setReverse();
      }
      highGear = !highGear;
    }

    drivetrain.move(controller.getLeftY(), controller.getRightX());
    SmartDashboard.putNumber("L ENCODER: ", drivetrain.getLIntegratedSensor());
    SmartDashboard.putNumber("R ENCODER: ", drivetrain.getRIntegratedSensor());
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
