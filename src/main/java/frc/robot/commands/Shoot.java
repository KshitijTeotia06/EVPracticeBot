// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  private boolean shooting = false;
  private double bumpertrim = 0;

  private NetworkTableEntry colorSensorEntry;
  private NetworkTableEntry limelightTargetEntry;
  private NetworkTableEntry limelightAimLocked;


  /** Creates a new Shoot. */
  public Shoot(Shooter shoot, Vision vision, Joystick stick, Intake intake, XboxController controller) {
    addRequirements(shoot);
    this.vision = vision;
    this.shoot = shoot;
    this.stick = stick;
    this.intake = intake;
    this.controller = controller;

    this.colorSensorEntry = Shuffleboard.getTab("Tokyo Drifter - Driver View").add("Ball Color", "Red").getEntry();
    this.limelightTargetEntry = Shuffleboard.getTab("Tokyo Drifter - Driver View").add("Limelight Target Found", true).getEntry();
    this.limelightAimLocked = Shuffleboard.getTab("Tokyo Drifter - Driver View").add("Limelight Aim Locked", true).getEntry();
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

    if (controller.getLeftBumperPressed()) {
      bumpertrim -= 100;
    } else if (controller.getRightBumperPressed()){
      bumpertrim += 100;
    }
    SmartDashboard.putNumber("trim value: ", bumpertrim);
    
    speed = shoot.computeV(vision.getY());
    SmartDashboard.putNumber("Difference Shooter Speed", speed - shoot.getRPM());
    SmartDashboard.putNumber("Y VALUE", vision.getY());


          Color colorReading = shoot.getColorSensorV3();

    if  (((shoot.teamColor.equals(DriverStation.Alliance.Red) && colorReading.red > colorReading.blue)) || ((shoot.teamColor.equals(DriverStation.Alliance.Blue) && colorReading.red < colorReading.blue))) {
      shoot.outakeV((speed + bumpertrim) * controller.getLeftTriggerAxis());
    }
    else {
      shoot.outakeV(0.5);
    }

    // SmartDashboard.putNumber("Current SHOOTER SPEED", shoot.getRPM());
    // SmartDashboard.putNumber("TARGET SPEED", speed);


    double outtakespeed= 0;
    if(controller.getRightTriggerAxis() > 0.1){
      // intake.conveyor(outtakespeed);
      outtakespeed = controller.getRightTriggerAxis();
      intake.ShootBalls(outtakespeed);
    }else if ((controller.getRightY() > 0.1) && (!intake.banner1Output() || !intake.banner2Output())){
      outtakespeed = controller.getRightY();
      intake.IntakeBalls(outtakespeed);
    }else if (controller.getRightY() < -0.1) {
      intake.transitionMotor(-0.5);
    }else{
      intake.conveyor(0);
      intake.intakeBrush(0);
      intake.transitionMotor(0);
    }
    // shoot.outtakeBall(controller.getLeftTriggerAxis());
  
    
    // SmartDashboard.putNumber("TRIGGER: ", stick.getY());
    // SmartDashboard.putNumber("SHOOTER SPEED", shoot.getRPM());

     SmartDashboard.putNumber("BlueColor Sensor Value", shoot.getColorSensorV3().blue);


    
     SmartDashboard.putNumber("RedColor Sensor Value", shoot.getColorSensorV3().red);

    // SmartDashboard.putNumber("COL SENSOR", shoot.getColorSensorV3());
    SmartDashboard.updateValues();

    // This makes the ball spit out the other teams ball


      // if (shoot.teamColor.equals(DriverStation.Alliance.Red) && colorReading.red > colorReading.blue) {
      //   shoot.outtakeBall(controller.getLeftTriggerAxis());
      // }
      // else if (shoot.teamColor.equals(DriverStation.Alliance.Blue) && colorReading.red < colorReading.blue) {
      //   shoot.outtakeBall(controller.getLeftTriggerAxis());
      // }
      // else {
      //   if (controller.getLeftTriggerAxis() > 0.1) {
      //     shoot.outtakeBall(0.5);
      //   }
      //   else {
      //     // This will happen if nothing gets pressed 
      //     shoot.outtakeBall(0);
      //   }
      // }
    
      if (colorReading.red > colorReading.blue) {
        colorSensorEntry.setString("Red");
      }
      else if (colorReading.red < colorReading.blue) {
        colorSensorEntry.setString("Blue");
      }
      else {
        colorSensorEntry.setString("Unknown");
      }

      if (vision.getTarget() == 1) {
        limelightTargetEntry.setBoolean(true);
      }
      else {
        limelightTargetEntry.setBoolean(false);
      }

      if ((vision.getX() > -0.1) && (vision.getX() < 0.1)) {
        limelightAimLocked.setBoolean(true);

      }
      else {
        limelightAimLocked.setBoolean(false);

      }
      
      
      
        Shuffleboard.update();

    // if (controller.getRightTriggerAxis() < 0.1) {
    //   if (intake.banner2Output()) {
    //     // Keeps the ball intake down
    //     intake.transitionMotor(-0.3);
    //   }
    // }
    // else {
    //   intake.transitionMotor(controller.getRightTriggerAxis());

    // }
   
    

    // SmartDashboard.putNumber("LEFT X", controller.getLeftTriggerAxis());
    // SmartDashboard.updateValues();
    
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
