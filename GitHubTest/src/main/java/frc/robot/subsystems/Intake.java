/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public Intake() {
  }

  private WPI_VictorSPX Intake1 = new WPI_VictorSPX(4);
  private WPI_VictorSPX Intake2 = new WPI_VictorSPX(1);

  public void setPowerIntake1(DoubleSupplier firstP){
    Intake1.set(firstP.getAsDouble());
  }

  public void setPowerIntake2(DoubleSupplier secondP){
    Intake2.set(secondP.getAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
