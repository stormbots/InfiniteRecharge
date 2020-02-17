/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.CAMERA_MOUNT_ANGLE;
import static frc.robot.Constants.CAMERA_MOUNT_HEIGHT;
import static frc.robot.Constants.SHOOTER_WHEEL_DIAMETER;
import static frc.robot.Constants.VISION_TARGET_HEIGHT;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public AHRS gyro;


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
  public Vision(AHRS navX) {
    this.gyro = navX;

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");
    targetHeading = navX.getAngle();
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

    distance = ((VISION_TARGET_HEIGHT-CAMERA_MOUNT_HEIGHT)/(Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE+y))));
    

    // post to smart dashboard periodically
    SmartDashboard.putNumber("vision/LimelightX", x);
    SmartDashboard.putNumber("vision/LimelightY", y);
    SmartDashboard.putNumber("vision/Distance to Target", distance);
    SmartDashboard.putNumber("vision/Gyro Current", gyro.getAngle());
    SmartDashboard.putNumber("vision/Gyro Target", targetHeading);
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
    table.getEntry("pipeline").setNumber(2); //0 for NON_3D
  }

  public void driverPipeline(){
    validTarget=false;
    table.getEntry("pipeline").setNumber(1);
  }

  public boolean isTargetValid(){
    return validTarget;
  }

  /** Returns distance, in Inches */
  public double getDistance(){
    return validTarget ? distance : 0;
  }

  /** Calculates and returns a target RPM needed to hit the  */
  public double getRPMForDistance(double distanceInInches){
    double diameter= SHOOTER_WHEEL_DIAMETER; //from Constants.java: You'll need this for the calculation so I stuffed it here to make it easier
    // TODO: Impliment me 
    double targetRPM = 1000;
    return targetRPM;
  }
  

}
