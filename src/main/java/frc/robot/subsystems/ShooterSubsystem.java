/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  CANSparkFlex motorLeft = new CANSparkFlex(ShooterConstants.shooterMotorLeftID, MotorType.kBrushless);
  CANSparkFlex motorRight = new CANSparkFlex(ShooterConstants.shooterMotorRightID, MotorType.kBrushless);
  CANSparkFlex motorFeeder = new CANSparkFlex(ShooterConstants.shooterMotorFeederID, MotorType.kBrushless);
  
  RelativeEncoder shooterEncoder = motorLeft.getEncoder();

  public ShooterSubsystem() {
    motorFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65000);
    motorFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65000);
    motorFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65000);
    motorFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65000);   
    motorFeeder.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65000);
  }

  @Override
  public void periodic() {}

  public void chargeShooter(double speed, PivotSubsystem pivotSubsystem)
  {
    runFeeder(0);
    if (!pivotSubsystem.isPivotVertical()) {
      runShooter(speed);
    } 
  }

  public void runFeederAndShooter() {
    runShooter(1);
    runFeeder(1);
  }

  public void shoot(double speed, PivotSubsystem pivotSubsystem) {
    if (pivotSubsystem.isPivotVertical()) {
      runShooter(0.2);
    } else {
      runShooter(speed);
    }
    
    if (isShooterAtSpeed() || pivotSubsystem.isPivotVertical()) {
      runFeeder(1);
    } 
  }

  public void stopAllMotors()
  {
    runFeeder(0);
    runShooter(0);
  }

  public void runFeeder(double speed)
  {
    motorFeeder.set(speed);
  }

  public void runShooter(double speed)
  {
    motorLeft.set(speed);
    motorRight.set(-speed);

    SmartDashboard.putNumber("shooter speed", shooterEncoder.getVelocity());
  }

  public boolean isShooterAtSpeed() {
    return shooterEncoder.getVelocity() > 5900;
  }

}
