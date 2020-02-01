/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  // motors
  CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax rotationMotor = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax translationMotor = new CANSparkMax(14, MotorType.kBrushless);

  CANEncoder armEncoder = new CANEncoder(armMotor);


  double length1 = 35;
  double length2 = 36;
  double maxAngle = 89;
  double minAngle = 0;
  double robotHeight = 18;

  double maxHeight = (Math.sin(maxAngle) * length1) + (Math.sin(maxAngle) * length2);
  MiniPID pid = new MiniPID(0.0,0,0);
  double targetheight = getHeight();

  
  public Climber() {
    armEncoder.setPositionConversionFactor(1);//TODO: Make encoder return arm angle
  }

  public void setTranslationPower(double translationSpeed){
    translationMotor.set(translationSpeed);
  }



  public void setHeight(double setpoint){
    targetheight = setpoint;
  }

  public double getHeight(){
    double theta = armEncoder.getPosition();
    return (Math.sin(theta) * length1) + (Math.sin(theta) * length2) + robotHeight;
  }

  

  public void setHookAngle(double degreesRelativeToArm){
    
  }

  public double getHookAngle(){
    return 0;
  }

  

  @Override
  public void periodic() {
    armMotor.set(pid.getOutput(getHeight(), targetheight));

    // This method will be called once per scheduler run
  }
}
