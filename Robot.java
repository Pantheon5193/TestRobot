/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kLeft = "Left";
  private static final String kMid = "Middle";
  private static final String kRight = "Right";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Motors
  private double indexStart = 0;
  private WPI_VictorSPX intake = new WPI_VictorSPX(4);
  private WPI_VictorSPX indexTrack = new WPI_VictorSPX(1);
  private WPI_VictorSPX fl = new WPI_VictorSPX(8);
  private WPI_VictorSPX fr = new WPI_VictorSPX(7);
  private WPI_VictorSPX bl = new WPI_VictorSPX(5);
  private WPI_VictorSPX br = new WPI_VictorSPX(6);
  private WPI_VictorSPX shooterTop = new WPI_VictorSPX(2);
  private WPI_VictorSPX shooterBottom = new WPI_VictorSPX(3);
  private Spark winchTop = new Spark(0);
  private Spark winchBottom = new Spark(1);
  private Spark wofMotor = new Spark(9);
  
  private Servo actuator = new Servo(2);
   
  

  //Sensors
  private double revs = 0;
  private int ballCounter = 0;
  private DigitalInput intakeSwitch = new DigitalInput(8);
  private DigitalInput indexSwitch = new DigitalInput(7);
  private DigitalInput shooterSwitch = new DigitalInput(6);
  private double shooterStage = 0;
  private double tinitial = 0;
  private boolean turning = false;
  private AHRS gyro = new AHRS(Port.kMXP);
  private Encoder leftEncoder  = new Encoder(1,2);
  private Encoder rightEncoder = new Encoder(3,4);
  private Timer timer = new Timer();
  private Timer timerAuton = new Timer();
  private Timer timerIndex = new Timer();
  private boolean driveStraightToggle = false;
  private double rpm;
  private double targetGyro=0;
  private boolean shooterToggle = false;
  private double Tp=.0325;
  private double Ti=.000001;
  private int a =0;
  private double autonInit = 0;
  private double Td=.0000001;
  private PIDController turnController = new PIDController(Tp, Ti, Td);
  private SpeedControllerGroup left = new SpeedControllerGroup(fl, bl);
  private SpeedControllerGroup right = new SpeedControllerGroup(fr, br);
  private DifferentialDrive tDrive = new DifferentialDrive(left, right);
  private boolean full = false;
  private Encoder shooterEncoder = new Encoder(0,5);
  private double time = 0;
  private DifferentialDriveOdometry oDrive = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getYaw()));
  


  //Controls
  private XboxController controller1 = new XboxController(0);

  //Robot Constants
  private double launchTargetX = -120;
  private double launchTargetY = 72;
  private double radius = 60;
    /**
   * This function is run when the robot is first started up and should be
   * 
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
    m_chooser.setDefaultOption("Left", kLeft);
    m_chooser.addOption("Middle", kMid);
    m_chooser.addOption("Right", kRight);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test. 
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override 
  public void robotPeriodic(){
   SmartDashboard.putNumber("Index Power", indexTrack.getBusVoltage());
   SmartDashboard.putNumber("Shooter Bottom Power", shooterBottom.getBusVoltage());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    timer.start();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.start();
    timerAuton.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    if(a==1){
     tDrive.tankDrive(1, 1);
     timer.delay(9);
     tDrive.tankDrive(0, 0);
     a=2;
    }
    if(a==0){
      if(timerAuton.get()>.25){
        time += .25;
        timerAuton.reset();
      }
      SmartDashboard.putNumber("Timer",timer.get());
      SmartDashboard.putNumber("Encoder", shooterEncoder.get());
      double ticksperRev = 8000;
      revs = shooterEncoder.getRaw()/ticksperRev;
      rpm = revs/time;
      
      double error = 70 - rpm;
  //77 is 14 feet
  //66 is 10 feet
      
      shooterBottom.set(.7);//.7 + .05*(error));
      shooterTop.set (-shooterBottom.get());
      if(error < 6 && autonInit==0){
        indexTrack.set(-.27);
        autonInit=timer.get() + 7;
      }
      if(error < 6 && timer.get() > autonInit ){
        indexTrack.set(0);
        shooterTop.set(0);
        shooterBottom.set(0);
        a=1;
      }
      }

    
  /*  
      double error = 66 - rpm;
      timer.start();
      while(error > 60){
        if(timer.get()>.25){
          time += .25;
          timer.reset();
        }
        
        double ticksperRev = 8000;
        revs = shooterEncoder.getRaw()/ticksperRev;
        rpm = revs/time;
        
        error = 66 - rpm;
        shooterBottom.set(.75 + .05*(error));
      shooterTop.set (-shooterBottom.get());
      }
      indexTrack.set(-.4);
      Timer.delay(4);
  
    switch (m_autoSelected) {
      case kRight:
      DifferentialDriveOdometry oDrive = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getYaw()));
      break;

      case kMid:
      oDrive = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getYaw()));
      break;

      case kLeft:
      oDrive = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getYaw()));
      break;
    }
    */
    /*
    switch (m_autoSelected) {
      case kRight:
        // Put custom auto code here
        intake.set(.1);  
        driveForward(5);
        rotatePID(45);
        intake.set(0);
        driveBackwards(10);
        while(timerAuton.get()<3){
          shooterPID(0.5);
        }
        shooter(0);
        rotatePID(0);
        
        break;
      case kMid:
        //robot starts at an angle to target        
        timerAuton.start();      
        while(timerAuton.get()<3){
            shooterPID(.5);
        }
        shooter(0);
        rotatePID(-45);
        gyro.reset();
        driveForward(8);
        rotatePID(45);
        driveForward(2);
        rotatePID(-170);
        driveBackwards(10);
        timerAuton.start();
        while(timerAuton.get()<4){
          shooterPID(.5);
        }
        shooter(0);

      break;
      case kLeft:
      default:
      timerAuton.start();
        while(timerAuton.get()<3){
          shooterPID(.5);
        }
        shooter(0);
        rotatePID(-30);
        driveForward(5);
        rotatePID(0);
        intake.set(.1);
        driveForward(10);
        driveBackwards(10);
        intake.set(0);
        rotatePID(-30);
        driveBackwards(5);
        rotatePID(0);
        while(timerAuton.get()<4){
          shooterPID(.5);
        }
        shooter(0);
        break;
    }
    */
  }
  @Override
  public void teleopInit() {
    
    timerAuton.start();
    gyro.reset();
    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.setDistancePerPulse(-2*Math.PI* 2/2048);
    rightEncoder.setDistancePerPulse(2*Math.PI* 2/2048);
    timerIndex.start();
    turnController.setIntegratorRange(-180, 180);
    timer.start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
    Pose2d pose = oDrive.update(Rotation2d.fromDegrees(-gyro.getYaw()), leftEncoder.getDistance(), rightEncoder.getDistance());
    if(turning){
    }else if(controller1.getBumper(Hand.kLeft)){
      driveStraightToggle = false;
      tDrive.tankDrive(.5* -Math.pow(controller1.getY(Hand.kLeft),3), .5* -Math.pow(controller1.getY(Hand.kRight),3));  
    }else if(controller1.getY(Hand.kLeft) < -.5 && controller1.getY(Hand.kRight) < -.5 && !driveStraightToggle){
      targetGyro = gyro.getYaw();
      driveStraightToggle = true;
    }else if(controller1.getY(Hand.kLeft) < -.5 && controller1.getY(Hand.kRight) < -.5 && driveStraightToggle) {
      double leftP =  -controller1.getY(Hand.kLeft) + ((targetGyro - gyro.getYaw())/90);
      double rightP = -controller1.getY(Hand.kLeft) -((targetGyro - gyro.getYaw())/90);
      tDrive.tankDrive(leftP, rightP);
    }else if(controller1.getY(Hand.kLeft) > .5 && controller1.getY(Hand.kRight) > .5 && !driveStraightToggle){
      targetGyro = gyro.getYaw();
      driveStraightToggle = true;
    }else if(controller1.getY(Hand.kLeft) > .5 && controller1.getY(Hand.kRight) > .5 && driveStraightToggle) {
      double leftP =  -controller1.getY(Hand.kLeft) + ((targetGyro - gyro.getYaw())/90);
      double rightP = -controller1.getY(Hand.kLeft) -((targetGyro - gyro.getYaw())/90);
      tDrive.tankDrive(leftP, rightP);
    }else{
      driveStraightToggle = false;
      tDrive.tankDrive(.95  *-Math.pow(controller1.getY(Hand.kLeft),3), -Math.pow(controller1.getY(Hand.kRight),3));
    } 
    SmartDashboard.putNumber("Drive Straight Target", targetGyro);
    
      //scale speed if .2 or more away 
      //
    
      if(controller1.getStartButtonPressed()){
        actuator.set(1);
        wofMotor.set(1);
      }  
      if(controller1.getBackButtonPressed()){
        actuator.set(0.5);
        wofMotor.set(.5);
      } 
       

    
    /*
    if(!shooterToggle){
    if(shooterSwitch.get()){
      if(!indexSwitch.get()){
        indexTrack.set(-.4);
        indexStart =timerIndex.get();
    }
  }else{
    indexStart=0;
  }
  if(timerIndex.get() > indexStart + .22){
    indexTrack.set(0);
  }
}else if(controller1.getAButton()){
  indexTrack.set(-.4);
}else{
  indexTrack.set(0); 
}
    */
    if(controller1.getAButton()){
      indexTrack.set(-.4);
    }else{
      indexTrack.set(0); 
    }
    


    
    

    
    double x = launchTargetX - pose.getTranslation().getX();
    double y = launchTargetY - pose.getTranslation().getY();
    turnController.setSetpoint(Math.toDegrees(Math.atan2(y, -x)));
      turnController.setTolerance(4);
    if(controller1.getYButtonPressed()){
      if(turning){
        turning=false;
      }else{
        turning=true;
      }
    }
    if(turning){
      x = launchTargetX - pose.getTranslation().getX();
      y = launchTargetY - pose.getTranslation().getY();
      
      
      left.set(turnController.calculate(gyro.getYaw()));
      right.set(turnController.calculate(gyro.getYaw()));
      if(Math.abs(turnController.getPositionError()) > 4 || turnController.getPositionError() < -4){
        tinitial = timerAuton.get();
      }else if(timerAuton.get() > tinitial+1){
        turning= false;
        shooterStage = 0;
      }
    }
    SmartDashboard.putNumber("Distance", Math.hypot(x, y) - radius);

    if(shooterStage ==1){
      if((Math.hypot(x, y) - radius) > 0){
        driveForward(Math.hypot(x, y) - radius);
      }else{
        driveBackwards(Math.hypot(x, y) - radius);
      }
    }
    if(!indexSwitch.get()&&!intakeSwitch.get()&&!shooterSwitch.get()){
      full = true;
    }
    else {
      full = false;
    }
    if(controller1.getTriggerAxis(Hand.kLeft) > .5 && full == false){
      intake.set(-0.5);
      }else if(controller1.getXButton()){
        intake.set(0.5);
      }else{
        intake.set(0);
      }

    if(controller1.getTriggerAxis(Hand.kRight) > .5){
      winchBottom.set(-controller1.getTriggerAxis(Hand.kRight));
      winchTop.set(-controller1.getTriggerAxis(Hand.kRight));
    }else if(controller1.getBButton()){
      winchTop.set(1);
      winchBottom.set(1);
    }else{
      winchBottom.set(0);
      winchTop.set(0);
    }

    

    

    
    if(controller1.getBumperPressed(Hand.kRight)){
      if(shooterToggle){
        shooterToggle=false;
      }else{
        shooterToggle=true;
    }
  }
    if(shooterToggle){
      if(timer.get()>.25){
        time += .25;
        timer.reset();
      }
      
      double ticksperRev = 8000;
      revs = shooterEncoder.getRaw()/ticksperRev;
      rpm = revs/time;
      
      double error = 77 - rpm;
  //77 is 14 feet
  //66 is 10 feet
      
      shooterBottom.set(.75 + .05*(error));
      shooterTop.set (-shooterBottom.get());
      }
      /*
      if(timer.hasPeriodPassed(.25)){
        time += .25;
      }
      
      double ticksperRev = 8000;
      revs = shooterEncoder.getRaw()/ticksperRev;
      rpm = revs/time;
      
      double error = 30 - rpm;
  
      shooterTop.set (-(.3 + .05*(error)));
      shooterBottom.set(.3 + .05*(error));
      SmartDashboard.putNumber("Shooter Encoder", shooterEncoder.getRaw());
    SmartDashboard.putNumber("Rpm",rpm);
    SmartDashboard.putNumber("Timer", time);
    SmartDashboard.putNumber("revs", revs);
      SmartDashboard.putBoolean("Fact", false);
      Timer.delay(1);
      SmartDashboard.putBoolean("Fact", true);
      if(Math.abs(error) < 5){
      for(int i=0;i<4;i++){
        indexTrack.set(-.4);
        Timer.delay(.2);
        indexTrack.set(0);
        Timer.delay(.2);
        if(i==4){
          shooterToggle=false;
        }
      }
    }
    */
    if(shooterToggle==false){
        shooterTop.set(0);
        shooterBottom.set(0);
        rpm=0;
        revs=0;
        time=0;
        shooterEncoder.reset();

      }

    SmartDashboard.putNumber("Gyro", gyro.getYaw());
    SmartDashboard.putBoolean("Index Swtich", indexSwitch.get());
    SmartDashboard.putBoolean("Shooter Swtich", shooterSwitch.get());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("X Position", pose.getTranslation().getX());
    SmartDashboard.putNumber("Y Position", pose.getTranslation().getY());
    SmartDashboard.putNumber("Ball Counter", ballCounter);
    SmartDashboard.putBoolean("ShooterToggle", shooterToggle);
    SmartDashboard.putNumber("Target Angle", turnController.getSetpoint());
    SmartDashboard.putNumber("Shooter Encoder", shooterEncoder.getRaw());
    SmartDashboard.putNumber("Rpm",rpm);
    SmartDashboard.putNumber("Timer", time);
    SmartDashboard.putNumber("revs", revs);
    SmartDashboard.putNumber("ShooterBottom Voltage", shooterBottom.getBusVoltage());
    SmartDashboard.putNumber("WinchTop Voltage", winchTop.getRaw());    
    SmartDashboard.putNumber("WinchNBottom Voltage", winchBottom.getRaw());

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void rotatePID(double targetAngle){
    turning = true;
    turnController.setSetpoint(targetAngle);
    while(turning){
    left.set(turnController.calculate(gyro.getYaw()));
    right.set(turnController.calculate(gyro.getYaw()));
      if(Math.abs(turnController.getPositionError()) > 4 || turnController.getPositionError() < -4){
        tinitial = timerAuton.get();
      }else if(timerAuton.get() > tinitial+1){
        turning= false;
        shooterStage = 0;
      }
    }
  }
  public void driveForward(double distance){
    double lefti = leftEncoder.getDistance();
    double righti = rightEncoder.getDistance();
    targetGyro = gyro.getYaw();
      double leftP =  .5 + ((targetGyro - gyro.getYaw())/90);
      double rightP = .5 -((targetGyro - gyro.getYaw())/90);
      tDrive.tankDrive(leftP, rightP);
    while(leftEncoder.getDistance() < lefti+distance && rightEncoder.getDistance() < righti+distance){
      Pose2d pose = oDrive.update(Rotation2d.fromDegrees(-gyro.getYaw()), leftEncoder.getDistance(), rightEncoder.getDistance());
      SmartDashboard.putNumber("LeftEncoder", leftEncoder.getDistance());
      SmartDashboard.putNumber("Left Compatative", lefti+distance);
      SmartDashboard.putNumber("Right Encoder",rightEncoder.getDistance());
      SmartDashboard.putNumber("Right Comparative", righti+distance);
      leftP  =  .5 + ((targetGyro - gyro.getYaw())/90);
      rightP = .5 -((targetGyro - gyro.getYaw())/90);
      tDrive.tankDrive(leftP, rightP);
    }
    tDrive.tankDrive(0, 0);
    shooterStage = 0;
  }
  public void driveBackwards(double distance){
    double lefti = leftEncoder.getDistance();
    double righti = rightEncoder.getDistance();
    targetGyro = gyro.getYaw();
      double leftP =  -.5 - ((targetGyro - gyro.getYaw())/90);
      double rightP = -.5 +((targetGyro - gyro.getYaw())/90);
      tDrive.tankDrive(leftP, rightP);
    while(leftEncoder.getDistance() > lefti-distance && rightEncoder.getDistance() > righti-distance){
      Pose2d pose = oDrive.update(Rotation2d.fromDegrees(-gyro.getYaw()), leftEncoder.getDistance(), rightEncoder.getDistance());
      SmartDashboard.putNumber("LeftEncoder", leftEncoder.getDistance());
      SmartDashboard.putNumber("Left Compatative", lefti+distance);
      SmartDashboard.putNumber("Right Encoder",rightEncoder.getDistance());
      SmartDashboard.putNumber("Right Comparative", righti+distance);
      leftP  =  -.5 - ((targetGyro - gyro.getYaw())/90);
      rightP = -.5 +((targetGyro - gyro.getYaw())/90);
      tDrive.tankDrive(leftP, rightP);
    }
    tDrive.tankDrive(0, 0);
    shooterStage = 0;
  }

  public void shooterPID(double targetSpeed){
    if(timer.hasPeriodPassed(.25)){
      time += .25;
    }
    
    double ticksperRev = 8000;
    revs = shooterEncoder.getRaw()/ticksperRev;
    rpm = revs/time;
    
    double error = 30 - rpm;

    shooterTop.set (-(targetSpeed + .05*(error)));
    shooterBottom.set(targetSpeed + .05*(error));
  }

  public void shooter(double power){
    
  }
}
