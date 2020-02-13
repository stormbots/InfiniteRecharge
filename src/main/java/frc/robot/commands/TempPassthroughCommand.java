/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Passthrough;

public class TempPassthroughCommand extends CommandBase {
  private Passthrough pt;
  DoubleSupplier power;

    
/**
   * Creates a new TempPassThroughCommand.
   */ 
  public TempPassthroughCommand(DoubleSupplier power,Passthrough pt) {
    this.pt = pt;
    this.power = power;
    addRequirements(pt);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    double newpower = MathUtil.clamp(power.getAsDouble(),-1.0,1.0);
    pt.tempmotor.set(newpower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
