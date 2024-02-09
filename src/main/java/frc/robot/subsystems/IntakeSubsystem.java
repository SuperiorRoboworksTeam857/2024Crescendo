/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  CANSparkFlex motorBack = new CANSparkFlex(IntakeConstants.intakeMotorBackID, MotorType.kBrushless);
  CANSparkFlex motorFront = new CANSparkFlex(IntakeConstants.intakeMotorFrontID, MotorType.kBrushless);
  

  public IntakeSubsystem() {}

  @Override
  public void periodic() {}

  public void runIntake(double speed)
  {
    motorBack.set(-speed);
    motorFront.set(-speed);
  }

}
