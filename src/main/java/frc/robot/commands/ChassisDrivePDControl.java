/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDrivePDControl extends CommandBase {
  private final Chassis chassis;
  private DoubleSupplier move;
  private DoubleSupplier turn;

  private CANPIDController pidLeft;
  private CANPIDController pidRight;

  // private SimpleMotorFeedforward leftFeedForward;
  // private SimpleMotorFeedforward rightFeedForward;

  private double lastLeftVelocity = 0;
  private double lastRightVelocity = 0;

  private double lastCycleTime;



  /**
   * Creates a new ChassisDriveManual.
   */
  public ChassisDrivePDControl(DoubleSupplier move, DoubleSupplier turn, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.move = move;
    this.turn = turn;
    addRequirements(chassis);

    pidLeft = chassis.getLeftLeadMotor().getPIDController();
    pidRight = chassis.getRightLeadMotor().getPIDController();

    pidLeft.setP(chassis.kP, 0);
    pidRight.setP(chassis.kP, 0);
    pidLeft.setD(chassis.kD, 0);
    pidRight.setD(chassis.kD, 0);

    pidLeft.setOutputRange(-chassis.maxChassisVelocity, chassis.maxChassisVelocity, 0);
    pidRight.setOutputRange(-chassis.maxChassisVelocity, chassis.maxChassisVelocity, 0);

    // leftFeedForward = new SimpleMotorFeedforward(chassis.kS, chassis.kV, chassis.kA);
    // rightFeedForward = new SimpleMotorFeedforward(chassis.kS, chassis.kV, chassis.kA);

    lastCycleTime = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    double forwardLinear = -move.getAsDouble();
    double turnLinear = turn.getAsDouble();

    double forwardSquared = Math.abs(forwardLinear) * forwardLinear;
    double turnSquared = Math.abs(turnLinear) * turnLinear;

    double[] tankDrivePowers = ourArcadeConversion(forwardLinear, turnLinear);


    double leftPower = tankDrivePowers[0];
    double rightPower = tankDrivePowers[1];


    double targetLeftVelocity = leftPower * chassis.maxChassisVelocity;
    double targetRightVelocity = rightPower * chassis.maxChassisVelocity;

    double leftAcceleration;
    double rightAcceleration;

    if(Timer.getFPGATimestamp() - lastCycleTime != 0) {
      leftAcceleration = (chassis.getLeftEncoder().getVelocity() - lastLeftVelocity) / (Timer.getFPGATimestamp() - lastCycleTime);
      rightAcceleration = (chassis.getRightEncoder().getVelocity() - lastRightVelocity) / (Timer.getFPGATimestamp() - lastCycleTime);
    }
    else {
      leftAcceleration = 0;
      rightAcceleration = 0;
    }

    double leftVoltageFF = chassis.kS * Math.signum(targetLeftVelocity) + chassis.kV * targetLeftVelocity + chassis.kA * leftAcceleration;
    double rightVoltageFF = chassis.kS * Math.signum(targetRightVelocity) + chassis.kV * targetRightVelocity + chassis.kA * rightAcceleration;

    lastLeftVelocity = chassis.getLeftEncoder().getVelocity();
    lastRightVelocity = chassis.getRightEncoder().getVelocity();

    lastCycleTime = Timer.getFPGATimestamp();


    // double leftFF = leftFeedForward.calculate(chassis.getLeftEncoder().getVelocity());
    // double rightFF = leftFeedForward.calculate(chassis.getLeftEncoder().getVelocity());

    // pidLeft.setFF(leftFeedForward.calculate(chassis.getLeftEncoder().getVelocity()), 0);
    // pidRight.setFF(rightFeedForward.calculate(chassis.getRightEncoder().getVelocity()), 0);

    pidLeft.setReference(targetLeftVelocity, ControlType.kVelocity, 0, leftVoltageFF);
    pidRight.setReference(targetRightVelocity, ControlType.kVelocity, 0, rightVoltageFF);

    // chassis.getLeftLeadMotor().pidWrite(leftPower);
    // chassis.getRightLeadMotor().pidWrite(rightPower);

    // chassis.drive.arcadeDrive(
    //   forwardLinear,
    //   turnLinear,
    //   false
    // );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    chassis.getLeftLeadMotor().set(0);
    chassis.getRightLeadMotor().set(0);
    // chassis.drive.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public double[] ourArcadeConversion(double turnInput, double forwardInput) {

    double inputForward = forwardInput;
    double inputTurn = turnInput;

    double leftPower = 0.0;
    double rightPower = 0.0;

    double maxInput = Math.copySign(
                                    Math.max(
                                      Math.abs(inputForward), 
                                      Math.abs(inputTurn)),
                                    inputForward
                                  );
    
    
    if(inputForward >= 0.0) {
      //First quadrant
      if(inputTurn >= 0.0) {
        leftPower = maxInput;
        rightPower = (inputForward - inputTurn);
      }
      //Second quadrant
      else {
        leftPower = (inputForward + inputTurn);
        rightPower = maxInput;
      }
    }
    else { 
      //Third quadrant
      if(inputTurn >= 0.0) {
        leftPower = (inputForward + inputTurn);
        rightPower = maxInput;
      }
      //Fourth quadrant
      else {
        leftPower = maxInput;
        rightPower = (inputForward - inputTurn);
      }
    }

    double[] powers = {leftPower, rightPower};

    return powers;
  }

}
