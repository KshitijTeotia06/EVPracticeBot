package frc.robot.subsystems;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private TalonFX intakeMotor;
  private VictorSPX brushMotor;
  private TalonSRX transitionMotor;
  // private Compressor compressor;

  private DigitalInput banner1; // dio 0
  private DigitalInput banner2; // dio 1
  // private DigitalInput banner3; // dio 2

  private boolean storageFull; // whether storage is full
  private int ballStored; // balls stored
  private String intakeStatus;
  private DoubleSolenoid leftSolenoid, rightSolenoid; 

  private boolean intakeDown;
  
  public Intake() {
    // compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR);
    brushMotor = new VictorSPX(Constants.BRUSH_MOTOR);
    transitionMotor = new TalonSRX(Constants.TRANSIT_MOTOR);
    intakeMotor.setInverted(true);
    transitionMotor.setInverted(true);
    // compressor.enableDigital();
    banner1 = new DigitalInput(Constants.BANNER_1);
    banner2 = new DigitalInput(Constants.BANNER_2);
    // banner3 = new DigitalInput(Constants.BANNER_3);
    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKEPISTON1_CHANNEL1, Constants.INTAKEPISTON1_CHANNEL2);
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKEPISTON2_CHANNEL1, Constants.INTAKEPISTON2_CHANNEL2);

    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
 
  public void intake(boolean isEjecting){
    if (!isEjecting) {

      if (!banner1Output()) {
        storageFull = true; // one slot open
        ballStored = 2;
      } else if (!banner2Output()) {
        storageFull = false; // two slots open 
        ballStored = 1;
      } else {
        storageFull = false;
        ballStored = 0;
      }

      if (storageFull) {
        intakeBall(0);
        intakeStatus = "intake motor not running";
      } else {
        intakeBall(.5);
        intakeStatus = "intake motor running";
       }
    }else{
      intakeBall(-1);
      transitionMotor(-1); // ejects everything 
      intakeStatus = "intake motor ejecting balls";
    } 

    SmartDashboard.putBoolean("Storage Full?", storageFull);
    SmartDashboard.putNumber("Balls Stored:", ballStored);
    SmartDashboard.putString("Intake Status:", intakeStatus);

  }

  public void intakeBall(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed); 
    SmartDashboard.putNumber("Intake: ", speed);
  }

  public void intakeBrush(double speed) {
    if (speed > 0.85) {
      speed = 0.85;
    }

    brushMotor.set(ControlMode.PercentOutput, speed);

  }

  public void transitionMotor(double speed) {
    transitionMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean banner1Output() { // banner sensor for 1st ball
    return banner1.get();
  }
  public boolean banner2Output() { // banner sensor for 2nd ball
    return banner2.get();
  }
  // public boolean banner3Output() { // banner sensor for transition
  //   return banner3.get();
  // }

  public void ejectBalls(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);

  }

  public void intakeToggle(){
    leftSolenoid.toggle();
    rightSolenoid.toggle();
    if ((leftSolenoid.get() == Value.kForward) && (rightSolenoid.get() == Value.kForward)){
      intakeDown = true;
    } else {
      intakeDown = false;
    }
  }

  public void intakeUp() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
  }

  public boolean getIntakeDown() {
    return intakeDown;
  }

}
