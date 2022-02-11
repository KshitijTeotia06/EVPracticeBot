package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private VictorSPX intakeMotor;
  private VictorSPX brushMotor;
  private TalonFX transitionMotor;
  private DigitalInput banner;
  private DigitalInput banner2;
  private DigitalInput banner3;
  private boolean ballLoaded;
  
  public Intake() {
    intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR);
    brushMotor = new VictorSPX(Constants.BRUSH_MOTOR);
    transitionMotor = new TalonFX(Constants.TRANSIT_MOTOR);

    banner = new DigitalInput(Constants.BANNER_1);
    banner2 = new DigitalInput(Constants.BANNER_2);
    banner3 = new DigitalInput(Constants.BANNER_3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /* 
  public void intake(boolean isEjecting){
    if (!isEjecting) {
      if (bannerOutput()) {
        ballLoaded = true;
      } else if (banner2Output()) {
        ballLoaded = false;
      }
      if (ballLoaded) {
        intakeBall(.5);
      } else {
        intakeBall(0);
       }
    }else{
      intakeBall(-1);
       }
  }
  */

  public void intakeBall(double speed){
    intakeMotor.set(ControlMode.PercentOutput, -speed); 
  }

  public void intakeBrush(double speed) {
    brushMotor.set(ControlMode.PercentOutput, speed);
  }

  public void transitionMotor(double speed) {
    transitionMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean bannerOutput() {
    return banner.get();
  }
  public boolean banner2Output() {
    return banner2.get();
  }

  public boolean banner3Output() {
    return banner3.get();
  }

  public void ejectBalls(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);

  }


}
