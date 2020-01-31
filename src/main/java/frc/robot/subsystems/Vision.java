/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public Gyro gyro;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");

  boolean validTarget = true;
  double x = 0;
  double y = 0;
  double targetHeading = 0;
  double distance = 0;

  /**
   * Creates a new Vision.
   */
  public Vision(Gyro gyro) {
    this.gyro = gyro;

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");
    targetHeading = gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read values periodically
    if (ta.getDouble(0.0) < 0.5 && ts.getDouble(0.0) >= -120 && ts.getDouble(0.0) <= -60) {
      // lights! useless!
    } else {
      // Good target!
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      targetHeading = gyro.getAngle() - x;
      validTarget = true;
    }

    if (validTarget == false) {
      return;
    }

    double distance = ((46.5-8.25)/(Math.tan(Math.toRadians(30+y)))); 
    // target height - mount height / tan(mounting angle in radians + error y)

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("Distance to Target", distance);
    SmartDashboard.putNumber("Gyro Current", gyro.getAngle());
    SmartDashboard.putNumber("Gyro Target", targetHeading);
  }

/**
 * @returns Heading to target (0...360). Allows for going past discontinuity (-X...360++)
 */
  public double getTargetHeading() {
    //TODO Make sure continuities are handled if we switch to navx
    return targetHeading;
  }

  public void lightsOn(){
    table.getEntry("ledMode").setNumber(3);
  }

  public void lightsOff(){
    table.getEntry("ledMode").setNumber(1);
  }

  public void targetPipeline(){
    table.getEntry("pipeline").setNumber(0);
  }

  public void driverPipeline(){
    table.getEntry("pipeline").setNumber(1);
  }
}
