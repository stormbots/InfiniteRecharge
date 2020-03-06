/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.Clamp;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  
  private final CANSparkMax shooterMotor = new CANSparkMax(11 ,MotorType.kBrushless);
  private final CANEncoder encoder = new CANEncoder(shooterMotor);
  private final CANPIDController sparkMaxPID = new CANPIDController(shooterMotor);



  // MiniPID feedForwardPID = new MiniPID(0,0,0,1/(5700.0*2));
  // MiniPID errorPID = new MiniPID(1/5700.0*1.2,0,0).setOutputLimits(-0.05, 0.5);
  private final SlewRateLimiter feedForwardSlew = new SlewRateLimiter( 1/2.0 ,0);

  double targetRPM = 0;

  // Not currently in use but may use later
  // SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.162,0.0641,0.0296);

  // Notifier notifier = new Notifier(()->runClosedLoop());



  public Shooter() {
    switch(Constants.botName){
      case COMP:
      break;

      case PRACTICE:
      default:
      break;
    }

    //old values
    //kU values
    // sparkMaxPID.setFF(1/(5700.0*1.9), 0);
    // sparkMaxPID.setP(1/5700.0*7, 0);
    
    // sparkMaxPID.setP(0.8*(1/5700.0*7));
    // sparkMaxPID.setD((1/5700.0*7) * 0.390243902439 /10.0);

    //gross test values for compression shooting
    // sparkMaxPID.setFF(1/(5700.0*1.2)*.8*.8*1.2*.8, 0); //runs at 10k rpm (over max)
    //sparkMaxPID.setP(0.8*0.8*(1/5700.0*2)*1.2); //tested pre comp
    sparkMaxPID.setP(0.8*0.8*(1/5700.0*2)*1.2);
    sparkMaxPID.setD(0.8*(1/5700.0*7) * 0.390243902439 /10.0);

    //weird comp retuning
    sparkMaxPID.setFF(1/(5700.0*1.2)*.8*.8*1.2*.8,0); //Correct for 7.5k rpm

    sparkMaxPID.setOutputRange(0, 1);
    //temp
    // sparkMaxPID.setP(0);
    // sparkMaxPID.setD(0);

    shooterMotor.setClosedLoopRampRate(0.1);

    // sparkMaxPID.setOutputRange(0, 1, 0);

    shooterMotor.setSmartCurrentLimit(45);
    // shooterMotor.setSmartCurrentLimit(80);
    


    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setOpenLoopRampRate(0.1);


    encoder.setPosition(0);
    encoder.setPositionConversionFactor(2);
    encoder.setVelocityConversionFactor(2);

    if(!SmartDashboard.containsKey("shooter/RMPDebugSet"))SmartDashboard.putNumber("shooter/RMPDebugSet", 1000);

    // notifier.startPeriodic(20);
  }

  public void reset() {
    // feedForwardPID.reset();
    // errorPID.reset();
    // shooterMotor.clearFaults();
    // We never use this???? 
  }

  public void setRPM(double rpm){
    this.targetRPM = rpm;
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  public boolean isOnTarget(){
    return Clamp.bounded(targetRPM, encoder.getVelocity()-100, encoder.getVelocity()+100);
  }

  // private void runClosedLoop() {
  //   double feedForwardOutput = feedForwardPID.getOutput(encoder.getVelocity(), targetRPM) ;
  //   double errorOutput = errorPID.getOutput(encoder.getVelocity(), targetRPM);


  //   // double feedForwardOutput = feedForward.calculate(targetRPM,0.0202);//todo: Accelleration

  //   if (encoder.getVelocity() < targetRPM*.5) {
  //     errorOutput = 0.0;
  //   }

    
  //   //TODO Feeder is geared to 1/10th the speed of shooter. Just run it as fast as possible now
  //   // and we'll be moving it to the Passthrough in a bit.

   
  //   feedForwardOutput = feedForwardSlew.calculate(feedForwardOutput);
  //   shooterMotor.set(feedForwardOutput + errorOutput);
  //   // feederMotor.set(feedForwardOutput + errorOutput);

  //   SmartDashboard.putNumber("shooter/contribFF", feedForwardOutput);
  //   SmartDashboard.putNumber("shooter/contribPID", errorOutput);

  // }

  //public void setMotorSpeed(double speed) {
  //  shooterMotor.set(speed);
  //}


  SlewRateLimiter rpmslew = new SlewRateLimiter(6000/1.0);
  @Override
  public void periodic() {

    //runClosedLoop();
    //sparkMaxPID.setReference(targetRPM, ControlType.kVelocity, 0, feedForward.calculate(targetRPM)/12.0, ArbFFUnits.kPercentOut);
    // sparkMaxPID.setReference(targetRPM, ControlType.kVelocity, 0);
    if (targetRPM == 0) { //Dan's fix
      shooterMotor.set(0);
    }
    else {
      sparkMaxPID.setReference(rpmslew.calculate(targetRPM), ControlType.kVelocity, 0);
    }
    // shooterMotor.set(0.5);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/RPM", encoder.getVelocity());
    SmartDashboard.putNumber("shooter/amps", shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/positons", encoder.getPosition());
    SmartDashboard.putNumber("shooter/appliedOutput", shooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("shooter/outputCurrent", shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/targetRPM", targetRPM);

    //Some debug values (all of them returned false always)
    SmartDashboard.putNumber("shooter/BUSVoltage", shooterMotor.getBusVoltage());
    SmartDashboard.putNumber("shooter/NeoHeat", shooterMotor.getMotorTemperature());

    SmartDashboard.putBoolean("fault/brownout", shooterMotor.getFault(FaultID.kBrownout));
    SmartDashboard.putBoolean("fault/motor", shooterMotor.getFault(FaultID.kMotorFault));
    SmartDashboard.putBoolean("fault/over current", shooterMotor.getFault(FaultID.kOvercurrent));
    SmartDashboard.putBoolean("fault/other", shooterMotor.getFault(FaultID.kOtherFault));
    SmartDashboard.putBoolean("fault/sensor", shooterMotor.getFault(FaultID.kSensorFault));
    SmartDashboard.putBoolean("fault/driver", shooterMotor.getFault(FaultID.kDRVFault));

    
  }
}
