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
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


public class JoystickDrive extends CommandBase {
private Timer timer = new Timer();
private DriveTrain m_driveTrain;
private DoubleSupplier leftP;
private DoubleSupplier rightP;
private boolean driveStraightToggle = false;
private double targetAngle=135;
private double targetDistance=24;
private int detectedColor = -1;
private int i = 0;
private double seen[] = new double[2];
private double detected[] = new double[2];
private double color[] = new double[2];
float Kp = -0.01f;
float steering_adjust;
double forward_adjust =0;
private VariableCommand vCommand;



/**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(DriveTrain subsystem, DoubleSupplier powerL, DoubleSupplier powerR, VariableCommand variablecCommand) {
    m_driveTrain = subsystem;
    leftP =  powerL;
    rightP=  powerR;
    vCommand = variablecCommand;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i=0;
    m_driveTrain.resetGyro();
    m_driveTrain.compensateGyro(0);
    m_driveTrain.resetEncoder();
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    seen = vCommand.seen;
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
  
    // steering_adjust = 0f;
    // if(x > 1.0){
    //   steering_adjust = Kp*-(float)x;
    // } else if (x<1.0){
    //   steering_adjust = Kp*-(float)x;
    // }
    //double error = area - 1;
    if(seen[0]!=0){
      vCommand.fin=false;
    }
    if(seen[0]==1) {
      if(seen[1]==2){
  
      }else if(seen[1] ==3) {
        
      }else if(seen[1] ==4) {
        
      }else if(seen[1]==0) {
        
      }
    }else if(seen[0]==2){
      if(seen[1]==1) {
        turn(90);
        driveForward(60);
      }else if(seen[1] ==3) {
        
      }else if(seen[1] ==4) {
        
      }else if(seen[1]==0) {
        
      }
    }else if(seen[0] ==3) {
      if(seen[1]==1) {

      }else if(seen[1]==2){
  
      }else if(seen[1] ==4) {
        turn(-90);
        driveForward(36);
      }else if(seen[1]==0) {
        
      }
    }else if(seen[0] ==4) {
      if(seen[1]==1) {

      }else if(seen[1]==2){
  
      }else if(seen[1] ==3) {
        
      }else if(seen[1]==0) {
        
      }
    }
    vCommand.seen[0] = 0;
    vCommand.seen[1] = 0;
    //seen[0]=0;
    //seen[1]=0;
    vCommand.fin = true;
    
    
    
    // if(area!=0 && area>1){//If target is seen and is far away
    //   forward_adjust = error*.25;
    //   m_driveTrain.setPower(() -> forward_adjust+ steering_adjust, () -> -forward_adjust+ steering_adjust);   
    // }else if(area !=0 && area<1){//If target is seen and close (notice the adjustment is more if close)
    //   forward_adjust = error;
    //   m_driveTrain.setPower(() -> forward_adjust+ steering_adjust, () -> -forward_adjust+ steering_adjust);   
    // }else{//If we are here then the target is not seen so we now want to use the controller
    //   if(((Math.abs(leftP.getAsDouble())>.5) && (Math.abs(rightP.getAsDouble())>.5)) && !driveStraightToggle && 
    // ((Math.round(leftP.getAsDouble()) + Math.round(rightP.getAsDouble()) == 0))){ //This long if statement checks if the sticks are pressed and sets the angle to automatically adjust to
    //   targetAngle = m_driveTrain.getAngle();
    //   driveStraightToggle=true;
    // }else if((Math.abs(leftP.getAsDouble())<.5) || (Math.abs(rightP.getAsDouble())<.5)){//If sticks are let go
    //   driveStraightToggle=false;
    // }
    // if((driveStraightToggle) && (leftP.getAsDouble()>0)){//Now this is auto correct with driving forward
    //   m_driveTrain.setPower(() ->((leftP.getAsDouble()) + ((targetAngle - m_driveTrain.getAngle())/90)),
    //    () -> (rightP.getAsDouble())+ ((targetAngle- m_driveTrain.getAngle())/90));

    // }else if((driveStraightToggle) && (leftP.getAsDouble()<0)){//Now this is auto correct with driving backward
    //   m_driveTrain.setPower(() ->(leftP.getAsDouble()/2+ ((targetAngle - m_driveTrain.getAngle())/90)),
    //    () -> (rightP.getAsDouble()/2) + ((targetAngle- m_driveTrain.getAngle())/90));
    // }else if(((Math.abs(leftP.getAsDouble())>.2) || (Math.abs((rightP.getAsDouble()))>.2)) && !driveStraightToggle){ //Cubed control if none of the above conditions apply but the sticks are pressed above a threshold
    //         m_driveTrain.setPower(() -> Math.pow((leftP.getAsDouble()),3), () -> Math.pow((rightP.getAsDouble()),3));
    // }else{//Obligitory turning it off
    //   m_driveTrain.setPower(() -> 0,() -> 0);
    // }
    // }

    SmartDashboard.putNumber("Steering adjustment: ", steering_adjust);
    SmartDashboard.putNumber("Movement adjustment", forward_adjust); 
    SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
    SmartDashboard.putBoolean("drive toggle",driveStraightToggle);
    SmartDashboard.putNumber("target angle", ((targetAngle- m_driveTrain.getAngle())/90));
    SmartDashboard.putNumber("Left P", Math.round(rightP.getAsDouble()));
    SmartDashboard.putNumber("Front Right", m_driveTrain.getEncoderCount()*-1.5 );
    SmartDashboard.putNumber("Lime X", x);
    SmartDashboard.putNumber("Lime y", y);
    SmartDashboard.putNumber("Lime A", area);
    SmartDashboard.putString("DRIVE COMMAND", " " + seen[0] + ", " + seen[1]);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.compensateGyro(0);
    m_driveTrain.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_driveTrain.compensateGyro(0);
    m_driveTrain.resetEncoder();
    return false;
  }

  private void driveForward(double distance){
    m_driveTrain.resetEncoder();
    timer.reset();
    timer.start();
    while(timer.get()<5){ 
      if(Math.abs(distance-(m_driveTrain.getEncoderCount()*-1.5)) > 1){
        if((distance-(m_driveTrain.getEncoderCount()*-1.5))>0){
          m_driveTrain.setPower(() ->.333, () -> -.333);
        }else{
          m_driveTrain.setPower(() ->-.1, () -> .1);
        }
      }else{
        m_driveTrain.setPower(() ->0, () -> 0);
      }
    }
    m_driveTrain.setPower(() ->0, () -> 0);
    timer.stop();
    timer.reset();
  }
  private void turn(double angle){
    double P = 0.005;
    double I = 0.0000065;
    double D = 0;//0.0000016;
    double error = 0;
    double pError = 0;
    double integral =0;
    timer.reset();
    timer.start();
    while(timer.get()<5){
      pError = error;
      error = angle - m_driveTrain.getAngle();
      integral += (error*.02);
      final double power = P*error + I*integral + D*((error-pError)/.02);
      m_driveTrain.setPower(() -> power, () -> power);  
      SmartDashboard.putNumber("Gyro", m_driveTrain.getAngle());
      SmartDashboard.putNumber("Gyro Error", error);
    }
    timer.stop();
    timer.reset();
    m_driveTrain.resetGyro();
    m_driveTrain.compensateGyro(-error);
  }
}
