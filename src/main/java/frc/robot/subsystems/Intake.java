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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  CANSparkMax sideMotor = new CANSparkMax(7,MotorType.kBrushless);
  CANSparkMax centerMotor = new CANSparkMax(8, MotorType.kBrushless);
  Solenoid solenoid = new Solenoid(2);
  private boolean UP;
  private boolean DOWN;

  private double SIDEMOTORSPEED;
  private double CENTERMOTORSPEED;
  
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
    switch(Constants.botName){
      case COMP:
        centerMotor.setInverted(true);
        sideMotor.setInverted(true);
  
        centerMotor.setSmartCurrentLimit(30);
        sideMotor.setSmartCurrentLimit(20);
        UP = false;
        DOWN = ! UP;
      break;
      case TABI:
      //no break: fallthrought to practice
      case PRACTICE:
        centerMotor.setInverted(true);
        sideMotor.setInverted(false);
  
        centerMotor.setSmartCurrentLimit(6, 20, 5);
        sideMotor.setSmartCurrentLimit(6, 20, 5);
        UP = false;
        DOWN = ! UP;
    }

    SIDEMOTORSPEED = 0.6;
    CENTERMOTORSPEED = 1.0;
    intakeUp();
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

  public void intakeOff(){
    sideMotor.set(0.0);
    centerMotor.set(0.0);
  }

  public void intakeReverse(){
    sideMotor.set(-SIDEMOTORSPEED);
    centerMotor.set(-CENTERMOTORSPEED);
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

    SmartDashboard.putNumber("intake/centerAmps", centerMotor.getOutputCurrent());
    SmartDashboard.putNumber("intake/sideAmps", sideMotor.getOutputCurrent());
    SmartDashboard.putNumber("intake/centerTemp", centerMotor.getMotorTemperature());
    SmartDashboard.putNumber("intake/sideTemp", sideMotor.getMotorTemperature());

  }
}
