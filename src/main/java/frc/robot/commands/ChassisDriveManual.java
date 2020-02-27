/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.CHASSIS_TURN_STATIC_FF;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;
import com.stormbots.PiecewiseLerp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveManual extends CommandBase {
  private final Chassis chassis;
  private DoubleSupplier move;
  private DoubleSupplier turn;
  private double JS_MIN=0.001;
  private double TURNFF=CHASSIS_TURN_STATIC_FF;

  //Map our control inputs to corresponding feedforwards 
  private PiecewiseLerp joystickMap = new PiecewiseLerp(
    new double[]{-1, -JS_MIN, 0, JS_MIN,  1},
    new double[]{-1, -TURNFF, 0, TURNFF, 1}
  );

  /**
   * Creates a new ChassisDriveManual.
   */
  public ChassisDriveManual(DoubleSupplier move, DoubleSupplier turn, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.move = move;
    this.turn = turn;
    addRequirements(chassis);
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

    turnSquared = joystickMap.getOutputAt(turnSquared);

    SmartDashboard.putNumber("chassisdrive/turnsquared", turnSquared);

    // chassis.drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);

    chassis.drive.arcadeDrive(
      forwardSquared,
      turnSquared,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    chassis.drive.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
