/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  CANSparkMax sideMotor = new CANSparkMax(7,MotorType.kBrushless);
  CANSparkMax centerMotor = new CANSparkMax(8, MotorType.kBrushless);
  Solenoid solenoid = new Solenoid(3);
  private final boolean UP = false;
  private final boolean DOWN = ! UP;

  private final double SIDEMOTORSPEED = 0.1;
  private final double CENTERMOTORSPEED = 0.1;
  
  //functions: 
  //engage/disengage
  //fix getting stuck:
    //ball stuck:
    // deactivates and lifts back up
    // if ball gets stuck to the point it lifting it cannot fix it reverse motors

    //mechanical issue
    // if it gets hit up it should automatically go down interupt
    // if turned off it should go up and turn off motors

    //outside isuse


  /**
   * Creates a new Intake.
   */
  public Intake() {

  }

  public void engage(){
    intakeDown();
    intakeOn();
  }

  public void disengage(){
    intakeUp();
    intakeOff();
    
  }

  public void intakeOn(){
    sideMotor.set(SIDEMOTORSPEED);
    centerMotor.set(CENTERMOTORSPEED);
  }
  // public void setMotorSpeed(double speed) {
  //   sideIntake.set(speed);
  //   frontIntake.set(speed);
  // }
  public void intakeOff(){
    sideMotor.set(0.0);
    centerMotor.set(0.0);
  }

  public void intakeUp() {
    solenoid.set(UP);
  }

  public void intakeDown() {
    solenoid.set(DOWN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
