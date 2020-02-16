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
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
  public CANSparkMax spoolMotor = new CANSparkMax(13, MotorType.kBrushless);

  CANEncoder armEncoder = new CANEncoder(armMotor);
  CANEncoder spoolEncoder = new CANEncoder(spoolMotor);

  //arm stuff
  double ARM_LENGTH_1 = 32.5;
  double ARM_LENGTH_2 = 22.5;

  double MAX_ARM_ANGLE = 89.0;
  double MIN_ARM_ANGLE = 0.0;

  double SPOOL_LENGTH = 38.0; //TODO find string length and spool circumfrence
  public double CLIMBER_BASE_HEIGHT = 24.5;
  // public double MAX_HEIGHT = (Math.sin(MAX_ARM_ANGLE) * ARM_LENGTH_1) + (Math.sin(MAX_ARM_ANGLE) * ARM_LENGTH_2);
  public double MAX_HEIGHT=SPOOL_LENGTH+CLIMBER_BASE_HEIGHT;
  // spool stuff

  double targetHeight; //initialize based off current in constructor

  MiniPID spoolPID = new MiniPID(0.0, 0, 0);
  MiniPID climbPID = new MiniPID(0.0,0,0);
 
  SlewRateLimiter climbSlew = new SlewRateLimiter(1,0);

  public boolean disable = false;
  
  public Climber() {
    switch(Constants.botName){
      case PRACTICE:
      break;
      case COMP:
      break;
    }
    armMotor.setInverted(true);
    spoolMotor.setInverted(true);
    
    armMotor.setIdleMode(IdleMode.kCoast); //TODO: Delete me
    spoolMotor.setIdleMode(IdleMode.kBrake); //TODO: Delete me

    armEncoder.setPositionConversionFactor(90/65.5);
    spoolEncoder.setPositionConversionFactor(38/1355.5);

    armEncoder.setPosition(0.0);
    spoolEncoder.setPosition(0.0);

    armMotor.setSmartCurrentLimit(20);
    spoolMotor.setSmartCurrentLimit(20);

    targetHeight = getSpoolHeight();
  }
  
  /* Main functions */
  
  public void setHeight(double targetHeight){
    this.targetHeight = targetHeight;
  }

  public double getHeight(){
    return getSpoolHeight();
  }


  /* Helper Functions */
  private double getSpoolHeight(){
    double currentSpoolHeight = spoolEncoder.getPosition()+CLIMBER_BASE_HEIGHT;
    return currentSpoolHeight; 
  }


  private double getArmHeight(){
    double theta = armEncoder.getPosition();
    theta = Math.toRadians(theta);

    return (Math.sin(theta) * ARM_LENGTH_1) + (Math.sin(theta) * ARM_LENGTH_2) + CLIMBER_BASE_HEIGHT;
  }

/*********** Periodic Stuff ********* */
  
   
  @Override
  public void periodic() {
    /* Height stuff */
    double spoolOutput = spoolPID.getOutput(getSpoolHeight(), targetHeight);
   
    // setspoolheight()
    // climbheight(spool.getheight())
    double armTargetHeight = getSpoolHeight();

    double armOutput = climbPID.getOutput(getArmHeight(), armTargetHeight);
    //FB based close loop
    armTargetHeight = MathUtil.clamp(armTargetHeight,0,80);
    armOutput = FB.fb(armTargetHeight, armEncoder.getPosition(), 0.07);
    armOutput = climbSlew.calculate(armOutput);

    // MathUtil.clamp(climbOutput, -0.01, 0.01); //safety clamp   
    spoolOutput = MathUtil.clamp(spoolOutput, -0.1, 0.1);
    armOutput = MathUtil.clamp(armOutput, -0.1, 0.1);
    if(!disable){
      spoolMotor.set(spoolOutput);
      //armMotor.set(armOutput);
    }

    SmartDashboard.putNumber("climb/encoderPos", armEncoder.getPosition());
    SmartDashboard.putNumber("climb/targetHeight", targetHeight);
    SmartDashboard.putNumber("climb/armHeight", getArmHeight());
    SmartDashboard.putNumber("climb/spoolHeight", getSpoolHeight());
    SmartDashboard.putNumber("climb/armAmpsOutput", armMotor.getOutputCurrent());
    // SmartDashboard.putNumber("spool/encoderPos",spoolEncoder.getPosition());
    // SmartDashboard.putNumber("climb/climbOutput", climbOutput);
   }
}
