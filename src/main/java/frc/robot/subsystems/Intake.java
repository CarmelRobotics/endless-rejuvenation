/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private SpeedController window;
  // private DigitalOutput roller;
  private VictorSP agitator = new VictorSP(IntakeConstants.AGITATOR_CHANNEL);
  private DigitalInput limit_switch = new DigitalInput(IntakeConstants.LIMIT_SWITCH);
  // private DigitalOutput feeder;
  // private DigitalInput bottomSwitch;
  // private DigitalInput topSwitch;
  public int ballsLoaded;
  public PWMVictorSPX roller;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    window = new VictorSP(IntakeConstants.WINDOW_CHANNEL);
    // roller = new DigitalOutput(IntakeConstants.ROLLER_CHANNEL);
    roller = new PWMVictorSPX(IntakeConstants.ROLLER_CHANNEL);
    // agitator = new Talon(IntakeConstants.AGITATOR_CHANNEL);
    // feeder = new DigitalOutput(IntakeConstants.FEEDER_CHANNEL);
    // bottomSwitch = new DigitalInput(IntakeConstants.BOTTOM_CHANNEL);
    // topSwitch = new DigitalInput(IntakeConstants.TOP_CHANNEL);
    // roller.enablePWM(0.375);
    // roller.setPWMRate(250);
    // feeder.enablePWM(0.375);
    // feeder.setPWMRate(250);
    ballsLoaded = 3;
    roller.setInverted(true);
  }
  public boolean getLimitSwitch() {
    return limit_switch.get();
  }
  public void move(double speed) {
    System.out.println("speen");
    window.set(speed);
  }
  public void stopWindow() {
    window.stopMotor();
  }
  public void in(double speed) {
    roller.setSpeed(speed);
  }
  public void stopRoller() {
    roller.stopMotor();
  }
  public void agitate(double speed) {
    agitator.set(speed);
  }
  public void stopAgitate() {
    agitator.stopMotor();
  }
  public void feed(double speed) {
    // feeder.updateDutyCycle((speed*0.5+1.5)/4);
  }
  // public boolean getBottomSwitch() {
  //   return bottomSwitch.get();
  // }
  // public boolean getTopSwitch() {
  //   return topSwitch.get();
  // }
  @Override
  public void periodic() {
    
  }
}
