// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public WPI_TalonFX l1, l2, r1, r2;
  public MotorControllerGroup l, r;
  public DifferentialDrive ddrive;
  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
  public DoubleSolenoid shifter;
  public Compressor c;

  // Auto
  
  public Encoder encoderL, encoderR;

  // public Solenoid shifterL, shifterR;
 public AHRS gyro = new AHRS(SPI.Port.kMXP);
 // public DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyroAngle)

  public Drivetrain() {
    // shifterL = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SHIFTER_L);
    // shifterR = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SHIFTER_R);
    l1 = new WPI_TalonFX(Constants.MOTOR_L1_ID);
    l2 = new WPI_TalonFX(Constants.MOTOR_L2_ID);
    r1 = new WPI_TalonFX(Constants.MOTOR_R1_ID);
    r2 = new WPI_TalonFX(Constants.MOTOR_R2_ID);
    c = new Compressor(0, PneumaticsModuleType.CTREPCM);
    c.enabled();
    c.enableDigital();
    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    l = new MotorControllerGroup(l1, l2);
    r = new MotorControllerGroup(r1, r2);
    l.setInverted(true);
    // shifter.set(Value.kReverse);
 
    // Auto start
    // encoderL = new Encoder(Constants.MOTOR_L1_ID, Constants.MOTOR_L2_ID);   
    // encoderR = new Encoder(Constants.MOTOR_R1_ID, Constants.MOTOR_R2_ID);   
    // Auto end

    // shifterL.set(false);
    // shifterR.set(false);
    shifter.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putBoolean("FWD", shifter.isFwdSolenoidDisabled());
    SmartDashboard.putBoolean("RWD", shifter.isRevSolenoidDisabled());
    SmartDashboard.updateValues();
    // SmartDashboard.putBoolean("compressor enabled: ", pcmCompressor.enabled());
    ddrive = new DifferentialDrive(l, r);
    
  }


  public void move(double power, double offset){ // power is the throttle (drive stick), offset is turning
    ddrive.arcadeDrive(power, offset);
  }

  public void setForward(){
    // shifterL.toggle();
    // shifterR.toggle();
    shifter.set(DoubleSolenoid.Value.kForward);
  }

  public void resetEncoders(){
    encoderL.reset();
    encoderR.reset();
  }

  public void setEncoderDis(double v){
    encoderL.setDistancePerPulse(v);
    encoderR.setDistancePerPulse(v);
  }

  public void setReverse(){
    // shifterL.toggle();
    // shifterR.toggle();
    shifter.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
