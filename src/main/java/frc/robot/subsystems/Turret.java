// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private TalonFX turretMotor;
  // private TalonSRX elevator = new TalonSRX(Constants.ELEVATOR);
  Vision vision;
  double kp;
  double kf, ki;
  double maxIntegral;
  double deadZone;
  double fric;
  double posErr;
  double intErr;
  double zerr;
  double motorOutput;
  // private DigitalInput limitSwitch_left;
  // private DigitalInput limitSwitch_right;

  /** Creates a new Turret. */
  public Turret(Vision vision) {
    turretMotor = new TalonFX(Constants.TURRT_MOTOR_ID);
    turretMotor.setInverted(true);
    this.vision = vision;
    // limitSwitch_left = new DigitalInput(Constants.LIMIT_LEFT);
    // limitSwitch_right = new DigitalInput(Constants.LIMIT_RIGHT);

    kp = .2;
    kf = 0*0.1;
    ki = 0*.0025;
    maxIntegral = 0*100;
    deadZone = 1;
    fric = 0;
    intErr = 0;
    posErr = 0;
    zerr = 0;
  }

  public void setSpeed(double speed){
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  public void turnTurret(double autoTrigger) {
    SmartDashboard.updateValues();
    if(vision.getTarget() != 1.0){
      turretMotor.set(ControlMode.PercentOutput, 0);
      return;
    }

    posErr = vision.getX();
    zerr = vision.getZ();

    intErr += posErr;

    fric = (kf / deadZone) * posErr;
    if (fric > kf) {
      fric = kf;
    }
    if (fric < -kf) {
      fric = -kf;
    }

    if (intErr > maxIntegral) {
      intErr = maxIntegral;
    }
    if (intErr < -1 * maxIntegral) {
      intErr = -1 * maxIntegral;
    }
    if(Math.abs(posErr) > .1 && posErr * intErr < 0){
      intErr = 0;
    }

    motorOutput = autoTrigger * (intErr * ki + posErr * kp + fric);
    // if (getLeftLimitSwitchStatus() == false && motorOutput < 0) {
      // motorOutput = .25;
    // }
    // if (getRightLimitSwitchStatus() == false && motorOutput > 0) {
      // motorOutput = -.25;
    // }
    turretMotor.set(ControlMode.PercentOutput, -motorOutput);

    SmartDashboard.putNumber("Vision X: ", posErr);
    SmartDashboard.putNumber("Vision Z: ", zerr);
    SmartDashboard.putNumber("TARGET FOUND", vision.getTarget());
    SmartDashboard.putNumber("VISION X", vision.getX());
    // SmartDashboard.putBoolean("LEFT LIMIT", getLeftLimitSwitchStatus());
    // SmartDashboard.putBoolean("RIGHT LIMIT", getRightLimitSwitchStatus());
    SmartDashboard.putNumber("Turret Motor Ouptut: ", motorOutput);
  }

  // public boolean getLeftLimitSwitchStatus() {// right
  //   return limitSwitch_left.get();
  // }

  // public boolean getRightLimitSwitchStatus() { // left
  //   return limitSwitch_right.get();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
