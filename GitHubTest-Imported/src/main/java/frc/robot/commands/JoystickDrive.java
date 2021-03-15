/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;
/**
 * An example command that uses an example subsystem.
 */
public class JoystickDrive extends CommandBase {
private DriveTrain m_driveTrain;
private DoubleSupplier leftP;
private DoubleSupplier rightP;


/**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(DriveTrain subsystem, DoubleSupplier powerL, DoubleSupplier powerR) {
    m_driveTrain = subsystem;
    leftP = powerL;
    rightP= powerR;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( (Math.abs(leftP.getAsDouble())>.2) || (Math.abs(rightP.getAsDouble())>.2)){
      m_driveTrain.setPower(leftP, rightP);
    }else{
      m_driveTrain.setPower(() -> 0,() -> 0);
    }

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
