/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.CAMERA_MOUNT_ANGLE;
import static frc.robot.Constants.CAMERA_MOUNT_HEIGHT;
import static frc.robot.Constants.SHOOTER_ANGLE;
import static frc.robot.Constants.SHOOTER_HEIGHT;
import static frc.robot.Constants.SHOOTER_WHEEL_DIAMETER;
import static frc.robot.Constants.VISION_TARGET_HEIGHT;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.PiecewiseLerp;

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
  NetworkTableEntry tv = table.getEntry("tv");

  boolean validTarget = true;
  double x = 0;
  double y = 0;
  double targetHeading = 0;
  double distance = 0;

  private double rpmForDistance = 0;

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
    tv = table.getEntry("tv");
    targetHeading = navX.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // post to smart dashboard periodically
    SmartDashboard.putNumber("vision/LimelightX", x);
    SmartDashboard.putNumber("vision/LimelightY", y);
    SmartDashboard.putNumber("vision/Distance to Target", distance);
    SmartDashboard.putNumber("vision/Gyro Current", gyro.getAngle());
    SmartDashboard.putNumber("vision/Gyro Target", targetHeading);
    SmartDashboard.putNumber("vision/rpmForDistance", rpmForDistance);

    SmartDashboard.putNumber("vision/rpm(13)", getRPMForDistance(12*13));
    SmartDashboard.putNumber("vision/rpm(15)", getRPMForDistance(12*15));
    SmartDashboard.putNumber("vision/rpm(17)", getRPMForDistance(12*17));
    SmartDashboard.putNumber("vision/rpm(19)", getRPMForDistance(12*19));
    SmartDashboard.putNumber("vision/rpm(21)", getRPMForDistance(12*21));
    SmartDashboard.putNumber("vision/rpm(23)", getRPMForDistance(12*23));


    if(tv.getDouble(0.0) == 0) {
      return;
    }

    // read values periodically
    if (ta.getDouble(0.0) < 0.5 && ts.getDouble(0.0) >= -120 && ts.getDouble(0.0) <= -60) {
      // lights! useless!
    } else {
      // Good target!
      x = tx.getDouble(0.0);
      y = ty.getDouble(0.0);
      targetHeading = gyro.getAngle() + x;
      validTarget = true;
    }

    if (validTarget == false) {
      return;
    }

    distance = ((VISION_TARGET_HEIGHT-CAMERA_MOUNT_HEIGHT)/(Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE+y)))+5);//5 is a MAGIC! just our weird error, accept it.

  }

/**
 * @returns Heading to target (0...360). Allows for going past discontinuity (-X...360++)
 */
  public double getTargetHeading() {
    //TODO Make sure continuities are handled if we switch to navx
    return targetHeading;
  }

  public void lightsOn(){
    // table.getEntry("ledMode").setNumber(3);
  }

  public void lightsOff(){
    // table.getEntry("ledMode").setNumber(1);
  }

  public void targetPipeline(){
    table.getEntry("pipeline").setNumber(0); //0 for NON_3D, 2 for 3D
  }
  public void targetPipelineFancy(){
    table.getEntry("pipeline").setNumber(2); //0 for NON_3D, 2 for 3D
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

    double distanceInFeet = distanceInInches/12;
    double height = (VISION_TARGET_HEIGHT - SHOOTER_HEIGHT)/12;

    double SHOOTER_ANGLE_RADIANS=Math.toRadians(SHOOTER_ANGLE);

    rpmForDistance = 
    //TODO: 3/pi is "simplified" form of rotations/feet and needs to be de-converted to use the actual wheel diameter constant
    //the three comes from 4 in being 1/3 of a foot...1/1/3 is 3. 

    (Math.sqrt( (16*Math.pow(distanceInFeet, 2)) / 
      (distanceInFeet*Math.cos(SHOOTER_ANGLE_RADIANS)*Math.sin(SHOOTER_ANGLE_RADIANS) - 
      ( (height) *Math.pow(Math.cos(SHOOTER_ANGLE_RADIANS), 2)) ) )
      )
      *(60*12*(1/(Math.PI*SHOOTER_WHEEL_DIAMETER)));

    rpmForDistance *= 2;
      //* (60) * (3/Math.PI);
      // /((SHOOTER_WHEEL_DIAMETER/2) * 0.10472);
    return rpmForDistance;
  }
  

  PiecewiseLerp distanceToRPM = new PiecewiseLerp(  
  new double[]{13*12, 15*12, 17*12, 19*12, 21*12, 23*12}, 
  new double[]{4450,  4700,   5000, 5350,   5650, 6000}
  );//PRACTICEBOT
  public double getDistanceEmpirical(double distance){
    return distanceToRPM.getOutputAt(distance);
  }
}
