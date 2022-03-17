// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
    drivetrain.setEncoderDis(1.0/256.0);
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    while (Math.abs(vision.getX()) < 0.02){
      turret.turnTurret(1);
    }
    double rpm = shooter.computeV(vision.getY());
    shooter.outakeV(rpm);
    while(Math.abs(shooter.getRPM() - rpm) <= 100){}
    
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
