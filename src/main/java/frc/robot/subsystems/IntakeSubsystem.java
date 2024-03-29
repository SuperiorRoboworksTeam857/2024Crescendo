/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  CANSparkFlex motorBack = new CANSparkFlex(IntakeConstants.intakeMotorBackID, MotorType.kBrushless);
  CANSparkFlex motorFront = new CANSparkFlex(IntakeConstants.intakeMotorFrontID, MotorType.kBrushless);
  

  public IntakeSubsystem() {
    motorBack.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65000);
    motorFront.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65000);

    motorBack.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65000);
    motorFront.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65000);

    motorBack.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65000);
    motorFront.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65000);
    
    motorBack.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65000);
    motorFront.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65000);
    
    motorBack.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65000);
    motorFront.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65000);
  }

  @Override
  public void periodic() {}

  private void runIntake(double speed)
  {
    motorBack.set(-speed);
    motorFront.set(-speed);
  }

  public void intake(PivotSubsystem pivotSubsystem)
  {
    if (pivotSubsystem.isPivotHorizontal()) {
      runIntake(0.5);
    }
  }

  public void outtake()
  {
    runIntake(-0.5);
  }

  public void stopIntake()
  {
    runIntake(0);
  }


}
