// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class AutoCommand extends CommandBase {
  /** Creates a new AutoCommand. */
  Drivetrain drivetrain;
  Turret turret;
  Vision vision;
  Shooter shooter;
  Intake intake;
  public AutoCommand(Drivetrain drivetrain, Turret turret, Vision vision, Shooter shooter, Intake intake) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.vision = vision;
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    // drivetrain.gyro.resetDisplacement();
    // drivetrain.gyro.reset();
    // drivetrain.setEncoderDis(1.0/256.0);
    // drivetrain.l.setInverted(true);

    
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.moveAuto(0.4, 4.5); // 6 ft
    // drivetrain.turnDegrees(90);
    // Shoots
    phase1();
    // // Drives backward
    // phase2();
    // // Shoots
    // phase1();
    
    // SmartDashboard.putNumber("L Encoder:", drivetrain.getIntegratedSensor());
    // SmartDashboard.updateValues();
    // intake.conveyor(0.5);
    // while (vision.getTarget() != 1.0){}
    // while (Math.abs(vision.getX()) < 0.02){
    //   turret.turnTurret(1);
    // }
    // double rpm = shooter.computeV(vision.getY());
    // shooter.outakeV(rpm);
    // while(Math.abs(shooter.getRPM() - rpm) <= 100){}
    // intake.transitionMotor(1);
  }

  public void phase1() { // shoot with auto aim
    Color colorReading = shooter.getColorSensorV3();
    // Checks the alliance color
    // if (shooter.teamColor.equals(DriverStation.Alliance.Red) && colorReading.red > colorReading.blue) {
    //   // Calculates RPM
    //   double rpm = shooter.computeV(vision.getY());

    //   // Starts Revving up motor
    //   shooter.outakeV(rpm);   
    // }
    // else if (shooter.teamColor.equals(DriverStation.Alliance.Blue) && colorReading.red < colorReading.blue) {
    //   // Calculates RPM
      double rpm = shooter.computeV(vision.getY());

      // Starts Revving up motor
      shooter.outakeV(rpm);   
    // }
    // else {
    //     shooter.outtakeBall(0.5);
    // }


    // Locks turret on target
    // turret.setSpeed(0);
    // turret.turnTurret(0.5);
    // Locks turret on target
    // while ((vision.getX() > 0.1) || (vision.getX() < -0.1)) {
    //   turret.turnTurret(1);
    // }
  
    // // This stops the turning
    // turret.turnTurret(0);   
    
    // Makes sure RPM meets minimum requirement
    // while (shooter.getRPM() < rpm) {}
    Timer.delay(3);
    
    // While there is still a ball loaded start transition(aka shoot)
    while (intake.banner2Output()){
      intake.transitionMotor(1);
    }

    // Stops shooter motor
    shooter.outtakeBall(0);
  }

  public void phase2() {

    intake.intakeToggle(); //drops intake

    // Gets the current start time
    double startTime = System.currentTimeMillis();

    // 6 second limit or banner 2 ouput become true will make this stop
    while (!intake.banner2Output() || (((System.currentTimeMillis() - startTime) / 1000.0) >= 6.0)) {
      // Run conveyor and move backward
      intake.conveyor(0.5);
      drivetrain.move(-0.3, 0);
    }

    // Stops drivetrain(safety)
    drivetrain.move(0, 0);

    // Pulls back up the intake
    intake.intakeToggle();

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
