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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

//import com.revrobotics.ColorSensorV3;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
/**
 * An example command that uses an example subsystem.
 */
public class JoystickIntake extends CommandBase {
private Intake m_Intake;
private DoubleSupplier leftTrig;
private DoubleSupplier rightTrig;
private BooleanSupplier holdA;
private BooleanSupplier holdB;
private BooleanSupplier pressA;
private DoubleSupplier rightStick;
private Timer timer = new Timer();
private I2C.Port i2cPort = I2C.Port.kOnboard;
private int detectedColor=-1;
private int i=1;
private double[] lastdetected = new double[2];
private double[] detected = new double[2];
public double[] command = new double[2];
private boolean fin = true;
private boolean commandready = true;
VariableCommand vCommand;

//private ColorSensorV3 IntakeTop = new ColorSensorV3(i2cPort);



/**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickIntake(Intake subsystem,  XboxController controller, VariableCommand variableCommand) {
    m_Intake  = subsystem;
    leftTrig  = () -> -controller.getTriggerAxis(GenericHID.Hand.kLeft);
    //rightStick = () -> controller.getY(GenericHID.Hand.kRight); Not Used
    holdA     = () -> controller.getAButton();
    holdB     = () -> controller.getBButton();
    pressA    = () -> controller.getAButtonPressed();
    rightTrig = () -> controller.getTriggerAxis(GenericHID.Hand.kRight);
    vCommand = variableCommand;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    lastdetected[0] = 0;
    lastdetected[1] = 0;
    detected[0] = 0;
    detected[1] = 0;
    command[0] = 0;
    command[1] = 0;
    i=1;
    commandready = true;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    if(i==5){
      i=1;
    }
    if(i==6 && vCommand.fin){
      i=1;
    }
    while(i<5){
      table.getEntry("pipeline").setNumber(i);
      detectedColor = i;
      i++;
      Timer.delay(.25);
      if(ta.getDouble(0.0) !=0 && detected[0] != detectedColor){
        if(commandready){
          timer.start();
          commandready=false;
        }
        lastdetected[0] = detected[0];
        lastdetected[1] = detected[1];
        detected[0] = detectedColor;
        detected[1] = tx.getDouble(0.0);
      }
      if(timer.get() >5){
        timer.stop();
        if(lastdetected[0]==0) {
          command[0] = detected[0];
          command[1] = 0;
        }else if(lastdetected[1]>detected[1]) {
          command[0] = detected[0];
          command[1] = lastdetected[0];
        }else{
          command[0] = lastdetected[0];
          command[1] = detected[0];
        }
        timer.reset();
        detected[0] = 0;
        lastdetected[0] = 0;
        commandready=true;
        vCommand.seen = command;
        i=6;
        
      }
      
    }
    
    SmartDashboard.putNumber("Detected:", detected[0]);
    SmartDashboard.putNumber("Currently detected at", detected[1]);
    SmartDashboard.putNumber("Last Detected:", lastdetected[0]);
    SmartDashboard.putNumber("Last detected at", lastdetected[1]);
    SmartDashboard.putString("Command", " " + command[0] + ", " + command[1]);
    SmartDashboard.putNumber("i", i);
    SmartDashboard.putNumber("Time", timer.get());
    SmartDashboard.putBoolean("FINISHED", vCommand.fin);
    
    
  // if(holdB.getAsBoolean()){ //This controls the intake rollers if the b button is pressed then it goes in reverse
  //   m_Intake.setPowerIntake1(() -> -1*leftTrig.getAsDouble());
  // } else {
  //   m_Intake.setPowerIntake1(() -> leftTrig.getAsDouble()*.375);
  // }
  // if(m_Intake.inputBottom()){ //&& (IntakeTop.getProximity()<135)){//If first sensor is triggered and second sensor is not run intake belt for .2 seconds
  //   m_Intake.setPowerIntake2(() -> -.3);
  //   Timer.delay(.2);
  //   m_Intake.setPowerIntake2(() -> 0);
  // }else if(pressA.getAsBoolean()){//Once A button is pressed turn on shooter for 1 second 
  //   m_Intake.setPowerShooter(() -> 1);
  //   Timer.delay(1);
  // }else if(holdA.getAsBoolean()){//After the previous operation is run the intake track will run if button is pressed
  //   m_Intake.setPowerIntake2(() -> -.5);
  // }else{//Otherwise turn off shooter
  //   m_Intake.setPowerIntake2(() -> 0);
  //   m_Intake.setPowerShooter(() -> 0);
  // }

  // SmartDashboard.putBoolean("Bottom Pressed", m_Intake.inputBottom());
  // //SmartDashboard.putNumber("Top Pressed", IntakeTop.getProximity());
  // SmartDashboard.putNumber("RPM", m_Intake.getRPM());   // ds
  
  
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
