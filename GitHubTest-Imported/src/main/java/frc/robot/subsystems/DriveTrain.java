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

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public DriveTrain() {
  }

  private WPI_VictorSPX fL = new WPI_VictorSPX(8);
  private WPI_VictorSPX fR = new WPI_VictorSPX(7);

  private WPI_VictorSPX bL = new WPI_VictorSPX(5);
  private WPI_VictorSPX bR = new WPI_VictorSPX(6);

  public void setPower(DoubleSupplier leftP, DoubleSupplier rightP){
    fL.set(leftP.getAsDouble());
    fR.set(rightP.getAsDouble());
    bL.set(leftP.getAsDouble());
    bR.set(rightP.getAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
