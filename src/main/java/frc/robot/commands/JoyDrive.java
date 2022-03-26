// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class JoyDrive extends CommandBase {
  /** Creates a new JoyDrive. */
  private final Drivetrain drivetrain;
  private Joystick stick, tstick;
  // private AHRS ahrsNavX;
  private boolean highGear = false;
  private boolean useTurnInPlace;
  private double sensScale = 1;

  // Auto
  private final IntakeBall intake;

  private NetworkTableEntry compressorEntry;
  private NetworkTableEntry shifterEntry;

  public JoyDrive(Drivetrain dt, Joystick stick, Joystick tstick, IntakeBall intake) { //replace parameters w (Drivetrain dt, Joystick dst, Joystick tst) for wheel and stick 
    this.drivetrain = dt;      
    this.stick = stick;
    this.tstick = tstick;
    this.intake = intake;
    
    // this.compressorEntry = Shuffleboard.getTab("Tokyo Drifter - Driver View").add("Compressor Current(AMP)", drivetrain.c.getCurrent()).getEntry();
    this.compressorEntry = Shuffleboard.getTab("Tokyo Drifter - Driver View").add("Compressor Full", determineIfCompressorIsFull()).getEntry();
    
    this.shifterEntry = Shuffleboard.getTab("Tokyo Drifter - Driver View").add("Shifter", getDrivetrainShifterState()).getEntry();
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
  public String getDrivetrainShifterState() {
    if (drivetrain.shifter.get() == DoubleSolenoid.Value.kForward) {
      return "Low";
    }
    else if (drivetrain.shifter.get() == DoubleSolenoid.Value.kReverse) {
      return "High";
    }
    else {
      return "Off";
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stick = new Joystick(Constants.DRIVE_STICK_PORT);
    tstick = new Joystick(Constants.TURN_STICK_PORT);
    drivetrain.resetEncoders();
    useTurnInPlace = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putData("shift gear", drivetrain.toggleGear());

    // SmartDashboard.putNumber("ENCODER", drivetrain.getIntegratedSensor());
    // SmartDashboard.updateValues();

    if (stick.getTriggerPressed()) {
      // SmartDashboard.putBoolean("CLICKED", true);
      // SmartDashboard.updateValues();
      if (highGear) {
        drivetrain.setForward();
      } else {
        drivetrain.setReverse();
      }
      highGear = !highGear;
    }
// Where da problem at? 
    // if (Math.abs(stick.getY()) < 0.1 && Math.abs(tstick.getX()) > 0.1) {
    //   //use the turn in place
    //   useTurnInPlace = true;
    // } else if (Math.abs(sti)) {
    //   useTurnInPlace = false;
    // }
    //((Math.abs(stick.getY()) < 0.1) ? true : false)
    
    drivetrain.move(-stick.getY() * sensScale, -tstick.getX(), ((Math.abs(stick.getY()) < 0.1) ? true : false));
//-1 * Math.signum(tstick.getX()) * Math.pow(Math.abs(tstick.getX()), 1.4)
    // Shuffleboard
    // compressorEntry.setNumber(drivetrain.c.getCurrent());

    // Updates shuffleboard to see if the compressor is full or not
    this.compressorEntry.setBoolean(determineIfCompressorIsFull());
    this.shifterEntry.setString(getDrivetrainShifterState());

    SmartDashboard.updateValues();
  }

  public Boolean determineIfCompressorIsFull() {
    if (drivetrain.c.getCurrent() == 0) {
      return true;
    } else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
