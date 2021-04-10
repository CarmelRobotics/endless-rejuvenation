/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.TurretConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.SerialPort;
public class Turret extends PIDSubsystem {
  private AHRS navx = new AHRS(SerialPort.Port.kUSB);
  
  private CANSparkMax shooterL;
  private CANSparkMax shooterR;
  private SpeedController turretAngleL, turretAngleR, susan;
  private SpeedControllerGroup shooter;
  // private DigitalOutput turret;
  private DigitalOutput turretWheel1;
  private DigitalOutput turretWheel2;
  private PWMSparkMax Shooter1;
  private PWMSparkMax Shooter2;
  private PWMVictorSPX elevator;
  private VictorSP windowMotor;
  private static double P = -1;
  private static double I = 0;
  private static double D = 0;
  private double lastEncoder = 0;
  private double zero = 0;
  AnalogEncoder encoder = new AnalogEncoder(new AnalogInput(EncoderConstants.SHOOTER_ENCODER_3_PORT));

  public Turret() {
    super(new PIDController(P, I, D));
    shooterL = new CANSparkMax(TurretConstants.SHOOTER_CAN_LEFT, MotorType.kBrushless);
    shooterR = new CANSparkMax(TurretConstants.SHOOTER_CAN_RIGHT, MotorType.kBrushless);
    windowMotor = new VictorSP(TurretConstants.WINDOW_PWM);
    
    //encoder.reset();
    
    // turret = new DigitalOutput(5);

   // Shooter1 = new PWMSparkMax(9);
    //Shooter2 = new PWMSparkMax(8);
    //Shooter1.setInverted(true);

    //turretWheel1 = new DigitalOutput(8);
    //turretWheel2 = new DigitalOutput(9);
    //turretWheel1.enablePWM(0);
    //turretWheel1.setPWMRate(200);
    //turretWheel2.enablePWM(0);
    //turretWheel2.setPWMRate(200);
    elevator = new PWMVictorSPX(TurretConstants.HOPPER2TURRET_PWM);
    setSetpoint(45);
    getController().setTolerance(1);
    rotateStop();
  }

  public double getNAVXAngle() {
    return navx.getAngle();
  }
  public void resetNAVX() {
    navx.reset();
  }
  // public double getAngleToTurnTo(double ballVel,double targetDist) {
  //   //ft per second
  //   double gravity = 32.2;
  //    double turretHeight = 19.25/12;
  //   double deltaY = targetHeight-turretHeight;
  //   double value1 = Math.sqrt(Math.pow(ballVel, 4)-gravity*(gravity*Math.pow(targetDist, 2)+2*deltaY*Math.pow(ballVel,2)));
  //   double angle1 = (Math.atan(ballVel-value1))/(gravity*targetDist);
  //   double angle2 = (Math.atan(ballVel+value1))/(gravity*targetDist);
  //   // if (ballVel > value1) {
  //   //   return (Math.atan(ballVel-value1))/(gravity*targetDist);
  //   // }else {
  //   //   return (Math.atan(ballVel+value1))/(gravity*targetDist);
  //   // }
  //   if (angle1>angle2) {
  //     return angle2;
  //   }else {
  //     return angle1;
  //   }
  // }

  public void shoot(){
    // shooter.set(1);
    elevator.set(-1);
    if (shooterL == null) {
       System.out.println("LEFT SHOOTER IS NULL");
     }else {
       shooterL.set(1);
       shooterR.set(-1);
     }

  }

  public void rotate(double speed){

    windowMotor.setSpeed(-speed);

  }
  public boolean isOutsideRange() {
    if (encoder.get() < 0.885) {
      return false;
    }else if (encoder.get()>0.01) { 
      return false;
    }else {
      return true;
    }
  }
  public double solveForDegrees(double dist) {
    return (-0.0208*Math.pow(dist,3))+(0.817*Math.pow(dist,2))+(-11*dist)+89.4;
  }

  public double getEncoderValue() {
    double value = -86.892*encoder.get();
    value = (value+lastEncoder)/2;
    lastEncoder = value;
    return value;
  }

  public void rotateStop(){

    windowMotor.setSpeed(0);
  }

  public void stop(){
    if (shooterL == null) {
       System.out.println("LEFT SHOOTER IS STILL NULL");
     }else {
       shooterL.set(0);
       shooterR.set(0);
     }
     elevator.stopMotor();
  }
  public void zeroEncoder() {
    encoder.reset();
  }
  public double getNAVXValue() {
    // System.out.println("ENCODERVALUE: " + encoder.get());
    return navx.getAngle();
  }
  
  public void shootDIO(double speed) {
    // turretWheel1.enablePWM((speed*0.5+1.5)/4);
    // turretWheel2.enablePWM((-speed*0.5+1.5)/4);
    // turretWheel1.updateDutyCycle((speed*0.5+1.5)/4);
    // turretWheel2.updateDutyCycle((-speed*0.5+1.5)/4);
    elevator.setSpeed(-1.0);
    Shooter1.set(speed);
    Shooter2.set(speed);
  }
  public void stopDIO() {
     // turretWheel1.disablePWM();
     // turretWheel2.disablePWM();
    Shooter2.stopMotor();
    Shooter2.stopMotor();
    elevator.stopMotor();
  }
  public void resetTurret() {
    encoder.reset();
  }
//   public void moveAtSpeed(double speed) {
//     turret.updateDutyCycle((speed*0.5+1.5)/4);
//   }
//   public void stopMovingTurret() {
//     turret.updateDutyCycle(0.0);
//   }
//  public int angle() {
//    turret.updateDutyCycle((0.25*0.5+1.5)/4);
//    return 0;
//  }
  // @Override
  // public void periodic() {
  //   SmartDashboard.putNumber("NAVX", getNAVXValue());
  //   // This method will be called once per scheduler run
  //   double testDist = 10;
  // }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    System.out.println("RUNNING PID");
    if (output < 0) {
      output /= 4;
    }
    windowMotor.set(output);
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return getEncoderValue();
  }
}
