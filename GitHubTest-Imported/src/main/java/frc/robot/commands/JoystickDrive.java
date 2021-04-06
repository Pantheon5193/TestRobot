/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
private boolean driveStraightToggle = false;
private double targetAngle=0;


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
    m_driveTrain.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(((Math.abs(leftP.getAsDouble())>.2) && (Math.abs(rightP.getAsDouble())>.2)) && !driveStraightToggle){
      targetAngle = m_driveTrain.getAngle();
      driveStraightToggle=true;
    }else if((Math.abs(leftP.getAsDouble())<.5) && (Math.abs(rightP.getAsDouble())<.5)){
      driveStraightToggle=false;
    }
    if((driveStraightToggle) && (leftP.getAsDouble()>0)){
      m_driveTrain.setPower(() ->((leftP.getAsDouble()/2)), //+ ((targetAngle - m_driveTrain.getAngle())/90)),
       () -> (rightP.getAsDouble()/2)); //+ ((targetAngle- m_driveTrain.getAngle())/90));

    }else if((driveStraightToggle) && (leftP.getAsDouble()<0)){
      m_driveTrain.setPower(() ->(leftP.getAsDouble()/2), //+ ((targetAngle - m_driveTrain.getAngle())/90)),
       () -> (rightP.getAsDouble()/2));// + ((targetAngle- m_driveTrain.getAngle())/90));
    }else if(((Math.abs(leftP.getAsDouble())>.2) || (Math.abs((rightP.getAsDouble()))>.2)) && !driveStraightToggle){
      m_driveTrain.setPower(() -> Math.pow((leftP.getAsDouble()),3), () -> Math.pow((rightP.getAsDouble()),3));
    }else{
      m_driveTrain.setPower(() -> 0,() -> 0);
    }
    SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
    SmartDashboard.putBoolean("drive toggle",driveStraightToggle);
    SmartDashboard.putNumber("target angle", ((targetAngle- m_driveTrain.getAngle())/90));
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
