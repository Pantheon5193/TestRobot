/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
/**
 * An example command that uses an example subsystem.
 */
public class JoystickIntake extends CommandBase {
private Intake m_Intake;
private DoubleSupplier firstP;
private BooleanSupplier isPressed;
private BooleanSupplier isPressed2;
private BooleanSupplier initialPress;
private Timer timer = new Timer();


/**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickIntake(Intake subsystem, DoubleSupplier power1,BooleanSupplier Ahold, BooleanSupplier Bpressed, BooleanSupplier Apressed) {
    m_Intake = subsystem;
    firstP = power1;
    isPressed=Ahold;
    isPressed2 = Bpressed;
    initialPress = Apressed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(isPressed2.getAsBoolean()){
    m_Intake.setPowerIntake1(() -> -1*firstP.getAsDouble());
  } else {
    m_Intake.setPowerIntake1(() -> firstP.getAsDouble()*.375);
  }
  if(m_Intake.input()){
    m_Intake.setPowerIntake2(() -> -.3);
    Timer.delay(.2);
    m_Intake.setPowerIntake2(() -> 0);
  }else if(initialPress.getAsBoolean()){
    m_Intake.setPowerShooter(() -> .85);
    Timer.delay(1);
  }else if(isPressed.getAsBoolean()){
    m_Intake.setPowerIntake2(() -> -.5);
  }else{
    m_Intake.setPowerIntake2(() -> 0);
    m_Intake.setPowerShooter(() -> 0);
  }

  SmartDashboard.putBoolean("Pressed", m_Intake.input());
  // ds
  
  
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
