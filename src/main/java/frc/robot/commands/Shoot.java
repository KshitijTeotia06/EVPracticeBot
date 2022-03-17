// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake;


public class Shoot extends CommandBase {
  private Shooter shoot;
  private Intake intake;
  private Joystick stick;
  private XboxController controller;
  private double throttle;
  double speed = 0;
  private boolean shooterWarmedUp = false;
  private Vision vision;

  /** Creates a new Shoot. */
  public Shoot(Shooter shoot, Vision vision, Joystick stick, Intake intake, XboxController controller) {
    addRequirements(shoot);
    this.vision = vision;
    this.shoot = shoot;
    this.stick = stick;
    this.intake = intake;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(stick.getRawButton(4)) { // starts warming up shooter (press again to stop shooter)
    //   if(shooterWarmedUp == false) {
    //     shooterWarmedUp = true;
    //   } else {
    //     shooterWarmedUp = false;
    //   }
    // }

    // if(shooterWarmedUp){
    //   shoot.outtakeBall(0.5); // keeps shooter spinning
    // }
    
    // if(controller.getLeftTriggerAxis() > 0){
    //   if(speed == 0) speed = 0.5 ;
    //   else speed = 0;
    // }
    
    // if(controller.getAButtonPressed()){
    //   speed = shoot.computeV(vision.getY());
    //   SmartDashboard.putNumber("Y VALUE", vision.getY());
    // }

    // // if ((shoot.getColorSensorV3().equals(Color.kBlue)) || (shoot.getColorSensorV3().equals(Color.kRed))) {
    //   shoot.outakeV(speed * controller.getLeftTriggerAxis());
    // // }
    // // else {
    //   // shoot.outakeV(0.1);
    // // }

    // SmartDashboard.putNumber("Current SHOOTER SPEED", shoot.getRPM());
    // SmartDashboard.putNumber("TARGET SPEED", speed);


    // double outtakespeed= 0;
    // if(controller.getRightTriggerAxis() > 0.1) outtakespeed = 1;
    // else outtakespeed = 0;
    // intake.transitionMotor(outtakespeed);
    // // shoot.outtakeBall(controller.getLeftTriggerAxis());
  
    
    // SmartDashboard.putNumber("TRIGGER: ", stick.getY());
    // SmartDashboard.putNumber("SHOOTER SPEED", shoot.getRPM());

    // // if (shoot.getColorSensorV3() == Color.kBlue) {
    // //   SmartDashboard.putString("Color Sensor Value", "Blue");


    // // }
    // // else if (shoot.getColorSensorV3() == Color.kRed) {
    // //   SmartDashboard.putString("Color Sensor Value", "Red");


    // // }
    // SmartDashboard.updateValues();

    shoot.outtakeBall(controller.getLeftTriggerAxis());
    intake.transitionMotor(controller.getRightTriggerAxis());

    SmartDashboard.putNumber("LEFT X", controller.getLeftTriggerAxis());
    SmartDashboard.updateValues();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.outtakeBall(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
