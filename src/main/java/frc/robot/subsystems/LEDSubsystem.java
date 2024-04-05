/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  DigitalInput beamBreak = new DigitalInput(9);
  Spark ledStrip = new Spark(0);
  
  public static final double BLUE_LIGHTS = 0.83;
  public static final double STROBE_RED = -0.11;
  public static final double GREEN_LIGHTS = 0.77; 

  boolean requestingNote = false;
  
  public LEDSubsystem() {}

  @Override
  public void periodic() {
    double lightPattern = BLUE_LIGHTS;

    if ((noteInFeeder())) {
      lightPattern = GREEN_LIGHTS;
    } else if (requestingNote) {
      lightPattern = STROBE_RED;
    } else {
      lightPattern = BLUE_LIGHTS;
    }

    ledStrip.set(lightPattern);
  }

  public void requestNote(){
    requestingNote = true;
  }

  public void dontRequestNote(){
    requestingNote = false;
  }

  public boolean noteInFeeder() {
    boolean noteIsInFeeder = !beamBreak.get();
    return noteIsInFeeder;
  }

  public boolean noteNotInFeeder() {
    return !noteInFeeder();
  }
}
