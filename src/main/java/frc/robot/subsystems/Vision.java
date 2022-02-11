// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  NetworkTable table;

  /*
  tv	Whether the limelight has any valid targets (0 or 1)
  tx	Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  ty	Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  ta	Target Area (0% of image to 100% of image)
  */

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta; 
  NetworkTableEntry tv; 
  // read values periodically
  double x;
  double y;
  double area;
  Boolean targets;
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
  }

  public double getX() {

    x = tx.getDouble(0) + 2.4;
    
    if(tv.getBoolean(false)){
      x = 0;
    }
    return x ;
    
  }

  // returns true or false depending on if a target is found
  public boolean getTarget() {

    targets = tv.getBoolean(false);
    return targets;

  }
}
