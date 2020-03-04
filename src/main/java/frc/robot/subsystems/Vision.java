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
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  public AHRS gyro;
  public MiniPID pidTurn;


  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");

  /**
   * Creates a new Vision.
   */
  public Vision(AHRS navX) {
    this.gyro = navX;

    pidTurn = new MiniPID(0,0,0);
    pidTurn.setSetpointRange(15); //TODO Find proper value //was 30
    pidTurn.setP(0.013);
    pidTurn.setI(0.001);
    // pidTurn.setD(0.0015);
    pidTurn.setMaxIOutput(0.15);
    pidTurn.setOutputLimits(0.35);
    pidTurn.setF((s,a,e)->{return Math.signum(e)*0.035;/*static FeedForward*/ });

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");
    tv = table.getEntry("tv");

    driverPipeline();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // post to smart dashboard periodically
    SmartDashboard.putBoolean("vision/targetValid", isTargetValid());
    SmartDashboard.putNumber("vision/Distance to Target", getDistance());
    SmartDashboard.putNumber("vision/Gyro Current", gyro.getAngle());
    SmartDashboard.putNumber("vision/Gyro Target", getTargetHeading());
    SmartDashboard.putNumber("vision/rpmForDistance", getDistanceToRPMEmpirical(getDistance()));

    SmartDashboard.putNumber("vision/xOffset", tx.getDouble(0.0));

    // SmartDashboard.putNumber("vision/rpm(13)", getRPMForDistance(12*13));
    // SmartDashboard.putNumber("vision/rpm(15)", getRPMForDistance(12*15));
    // SmartDashboard.putNumber("vision/rpm(17)", getRPMForDistance(12*17));
    // SmartDashboard.putNumber("vision/rpm(19)", getRPMForDistance(12*19));
    // SmartDashboard.putNumber("vision/rpm(21)", getRPMForDistance(12*21));
    // SmartDashboard.putNumber("vision/rpm(23)", getRPMForDistance(12*23));


  }

/**
 * @returns Heading to target (0...360). Allows for going past discontinuity (-X...360++)
 */
  public double getTargetHeading() {
    double targetHeading = gyro.getAngle() + tx.getDouble(0.0);
    return targetHeading;
  }
  public double getTargetOffset() {
    return tx.getDouble(0.0);
  }

  public void lightsOn(){
    // table.getEntry("ledMode").setNumber(3);
  }

  public void lightsOff(){
    // table.getEntry("ledMode").setNumber(1);
  }

  public void targetPipeline(){
    table.getEntry("pipeline").setNumber(0); //0 for NON_3D, 2 for 3D
    lightsOn();
  }
  public void targetPipelineFancy(){
    table.getEntry("pipeline").setNumber(2); //0 for NON_3D, 2 for 3D
    lightsOn();
  }

  public void driverPipeline(){
    table.getEntry("pipeline").setNumber(1);
    lightsOff();
  }

  public boolean isTargetValid(){
    if( tv.getDouble(0.0)<1 ) return false;

    // read values periodically
    // if (ta.getDouble(0.0) < 0.5 && ts.getDouble(0.0) >= -120 && ts.getDouble(0.0) <= -60) {
    //   // lights! useless!
    //   return false;
    // } 
    //good target!
    return true;
  }

  /** Returns distance, in Inches */
  public double getDistance(){
    double y = ty.getDouble(20.0 * 12);
    double distance = (VISION_TARGET_HEIGHT - CAMERA_MOUNT_HEIGHT) / Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE + y));
    distance += 5;//5 is a MAGIC! just our weird error, accept it.
    return distance;
  }

  /** Calculates and returns a target RPM needed to hit the target */
  public double getRPMForDistance(double distanceInInches){

    double distanceInFeet = distanceInInches/12;
    double heightInFeet = (VISION_TARGET_HEIGHT - SHOOTER_HEIGHT)/12;

    double SHOOTER_ANGLE_RADIANS=Math.toRadians(SHOOTER_ANGLE);

    double rpmForDistance = 
    //TODO: 3/pi is "simplified" form of rotations/feet and needs to be de-converted to use the actual wheel diameter constant
    //the three comes from 4 in being 1/3 of a foot...1/1/3 is 3. 

    (Math.sqrt( (16*Math.pow(distanceInFeet, 2)) / 
      (distanceInFeet*Math.cos(SHOOTER_ANGLE_RADIANS)*Math.sin(SHOOTER_ANGLE_RADIANS) - 
      ( (heightInFeet) *Math.pow(Math.cos(SHOOTER_ANGLE_RADIANS), 2)) ) )
      )
      *(60*12*(1/(Math.PI*SHOOTER_WHEEL_DIAMETER)));

    rpmForDistance *= 2;
      //* (60) * (3/Math.PI);
      // /((SHOOTER_WHEEL_DIAMETER/2) * 0.10472);
    return rpmForDistance;
  }
  

  public double getDistanceToRPMEmpirical(double distance){
    return Constants.distanceToRPM.getOutputAt(distance);
  }

  public boolean isOnTarget(double degrees) {
    if(isTargetValid() && Math.abs(getTargetOffset()) <= degrees) {
      return true;
    }
    return false;
  }
}
