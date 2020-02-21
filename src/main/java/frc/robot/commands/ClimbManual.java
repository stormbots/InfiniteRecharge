/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * An example command that uses an example subsystem.
 */
public class ClimbManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Climber Climber;
  private frc.robot.subsystems.Climber climber;
  private DoubleSupplier joystickValue;
  /** Safety check to avoid joystick errors*/
  private boolean enableSafety = false;
  /** Allow a button or game timer to enable */
  private BooleanSupplier enableOperator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbManual(BooleanSupplier enable, DoubleSupplier targetHeight, Climber climber) {
    this.climber = climber;
    this.joystickValue = targetHeight;
    this.enableOperator = enable;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //ensure joystick is where we need it before running
    if(joystickValue.getAsDouble()>0.9)enableSafety=true;
    if(enableSafety==false)return;
    //ensure operator and/or game timer allows this to run
    if(enableOperator.getAsBoolean()==false)return;

    double height = Lerp.lerp(joystickValue.getAsDouble(), 1, -1, climber.CLIMBER_BASE_HEIGHT, climber.MAX_HEIGHT );
    climber.setHeight(height);
    SmartDashboard.putNumber("climb/joystickTargetHeight", height);
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
