/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // private static TalonFX intakeMotor1 = new TalonFX(Constants.INTAKE_1);
  // private static TalonFX intakeMotor2 = new TalonFX(Constants.INTAKE_2);
  // private static TalonFX turretMotor = new TalonFX(Constants.TURRET);
  private static TalonSRX shooterMotor1;
  private static TalonSRX shooterMotor2;
  //private static VictorSPX intakeMotor = new VictorSPX(Constants.INTAKE);
  private DigitalInput banner;
  boolean ballLoaded;
  private DigitalInput banner2;

  private double currentSpeed;
  private double maxSpeed;

  Timer timer;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterMotor1 = new TalonSRX(Constants.SHOOTER1);
    shooterMotor2 = new TalonSRX(Constants.SHOOTER2);
    shooterMotor2.setInverted(true);
    shooterMotor2.set(ControlMode.Follower, Constants.SHOOTER1);
    banner = new DigitalInput(6);
    banner2 = new DigitalInput(8);
    ballLoaded = false;
    timer = new Timer();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getShooterVel() {
  
    return (shooterMotor1.getSelectedSensorVelocity() + shooterMotor2.getSelectedSensorVelocity()) / 2;
  }

  public void outtakeBall(double speed) {
    shooterMotor1.set(ControlMode.Velocity, speed * 7500);
    // shooterMotor1.set(ControlMode.PercentOutput, speed);
    // shooterMotor2.set(ControlMode.PercentOutput, speed);
    SmartDashboard.getNumber("Shooter speed: ", getShooterVel());

  }

  public void inttakeBall(double speed) {
    //intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean bannerOutput() {
    return banner.get();

  }

  public boolean banner2Output() {
    return banner2.get();
  }

  public void ejectBalls(double speed) {
    //intakeMotor.set(ControlMode.PercentOutput, speed);

  }

  public void intake(boolean isEjecting) {
   // System.out.println("Is ejecting: " + isEjecting);
    if (!isEjecting) {
      if (bannerOutput()) {
        ballLoaded = true;
        timer.start();
      } else if (banner2Output()) {
        ballLoaded = false;
      }
      if(timer.get() > .5){
        ballLoaded = false;
      }
      if (ballLoaded) {
        inttakeBall(0.5);
      } else {
        inttakeBall(0);
      }
    }else{
      inttakeBall(-1);
    }
  }

}