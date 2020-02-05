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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  // motors
  CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax hookMotor = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax translationMotor = new CANSparkMax(14, MotorType.kBrushless);

  CANEncoder armEncoder = new CANEncoder(armMotor);
  CANEncoder hookEncoder = new CANEncoder(hookMotor);

  /* Constants for climber */
  double length1 = 35;
  double length2 = 36;
  double maxAngle = 89;
  double minAngle = 0;
  double robotHeight = 18;
  double maxHeight = (Math.sin(maxAngle) * length1) + (Math.sin(maxAngle) * length2);

  MiniPID armRotationPID = new MiniPID(0,0,0);
  MiniPID climbPID = new MiniPID(0.0,0,0);
  double targetheight = getHeight();
  double hookTargetAngle = getHookAngle();

  
  public Climber() {
    armMotor.setInverted(true);
    armEncoder.setPositionConversionFactor(1);//TODO: Make encoder return arm angle
    hookEncoder.setPositionConversionFactor(1);//TODO: Make encoder return arm angle
  }

/*********** Translation Stuff ********* */

  public void setTranslationPower(double translationSpeed){
    translationMotor.set(translationSpeed);
  }

/*********** Height Stuff ********* */


  public void setHeight(double setpoint){
    targetheight = setpoint;
  }

  public double getHeight(){
    double theta = armEncoder.getPosition();
    return (Math.sin(theta) * length1) + (Math.sin(theta) * length2) + robotHeight;
  }

/*********** Hook Stuff ********* */

  public void setHookAngle(double degreesRelativeToArm){
    hookTargetAngle = degreesRelativeToArm;
  }

  public double getHookAngle(){
    return hookEncoder.getPosition();
  }


/*********** Periodic Stuff ********* */

  @Override
  public void periodic() {
    /* Height stuff */
    double climbOutput = climbPID.getOutput(getHeight(), targetheight);
    // SmartDashboard.putNumber("climb/climbOutput", climbOutput);
    MathUtil.clamp(climbOutput, 0.1, 0.1);
    armMotor.set(climbOutput);
    
    /* Hook Stuff */ 
    double hookOutput = armRotationPID.getOutput(getHookAngle(), hookTargetAngle);
    MathUtil.clamp(hookOutput,0.1,0.1);
    hookMotor.set(hookOutput);


    // This method will be called once per scheduler run
  }
}
