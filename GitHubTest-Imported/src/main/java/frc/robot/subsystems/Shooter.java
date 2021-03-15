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

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
  }

  private WPI_VictorSPX shooterTop = new WPI_VictorSPX(2);
  private WPI_VictorSPX shooterBottom = new WPI_VictorSPX(3);

  public void setPowerShooter(DoubleSupplier firstP){
    shooterTop.set(-firstP.getAsDouble());
    shooterBottom.set(firstP.getAsDouble());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
