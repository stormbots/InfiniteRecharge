/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.closedloop.FB;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.commands.ClimberSetTranslation;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  // motors
  CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
  public CANSparkMax hookMotor = new CANSparkMax(13, MotorType.kBrushless);

  //public CANSparkMax translationMotor = new CANSparkMax(14, MotorType.kBrushless);



  CANEncoder armEncoder = new CANEncoder(armMotor);
  CANEncoder hookEncoder = new CANEncoder(hookMotor);

  /* Constants for climber */
  double length1 = 32.5;
  double length2 = 28.125;
  double maxAngle = 89.0;
  double minAngle = 0.0;
  double robotHeight = 24.5;
  double maxHeight = (Math.sin(maxAngle) * length1) + (Math.sin(maxAngle) * length2);

  MiniPID armRotationPID = new MiniPID(0,0,0);
  MiniPID climbPID = new MiniPID(0.0,0,0);
  double targetheight = getHeight();
  double hookTargetAngle = getHookAngle();

  
  public Climber() {
    armMotor.setInverted(true);
    armMotor.setSmartCurrentLimit(25);
    armEncoder.setPositionConversionFactor(90.0 / 65.5);//TODO: Make encoder return arm angle
    hookEncoder.setPositionConversionFactor(1);//TODO: Make encoder return arm angle

    armMotor.setIdleMode(IdleMode.kCoast); //TODO: Delete me
    hookMotor.setIdleMode(IdleMode.kCoast); //TODO: Delete me

    armEncoder.setPosition(0.0);
  }

/*********** Translation Stuff ********* */

  public void setTranslationPower(double translationSpeed){
    // translationMotor.set(translationSpeed);
  }

/*********** Height Stuff ********* */


  public void setHeight(double setpoint){
    targetheight = setpoint;
  }

  public double getHeight(){
    double theta = armEncoder.getPosition();
    theta = Math.toRadians(theta);
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
  SlewRateLimiter climbSlew = new SlewRateLimiter(1,0);
   
  @Override
  public void periodic() {
    /* Height stuff */
    double climbOutput = climbPID.getOutput(getHeight(), targetheight);
    // SmartDashboard.putNumber("climb/climbOutput", climbOutput);
    MathUtil.clamp(climbOutput, -0.01, 0.01);

    //semi debuggery
    targetheight = MathUtil.clamp(targetheight,0,80);
    climbOutput = FB.fb(targetheight, armEncoder.getPosition(), 0.07);
    climbOutput = MathUtil.clamp(climbOutput, -0.1, 0.1);
    climbOutput = climbSlew.calculate(climbOutput);
    armMotor.set(climbOutput);

    SmartDashboard.putNumber("climb/encoderPos", armEncoder.getPosition());
    SmartDashboard.putNumber("climb/targetHeight", targetheight);
    SmartDashboard.putNumber("climb/height", getHeight());
    SmartDashboard.putNumber("climb/armAmpsOutput", armMotor.getOutputCurrent());

    
    /* Hook Stuff */ 
    double hookOutput = armRotationPID.getOutput(getHookAngle(), hookTargetAngle);
    MathUtil.clamp(hookOutput,-0.01,0.01);
    // hookMotor.set(hookOutput);


    // This method will be called once per scheduler run
  }
}
