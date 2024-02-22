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
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  CANSparkFlex motorLeft = new CANSparkFlex(ShooterConstants.shooterMotorLeftID, MotorType.kBrushless);
  CANSparkFlex motorRight = new CANSparkFlex(ShooterConstants.shooterMotorRightID, MotorType.kBrushless);
  CANSparkFlex motorFeeder = new CANSparkFlex(ShooterConstants.shooterMotorFeederID, MotorType.kBrushless);
  

  public ShooterSubsystem() {}

  @Override
  public void periodic() {}

  public void chargeShooter(double speed, PivotSubsystem pivotSubsystem)
  {
    runFeeder(0);
    if (!pivotSubsystem.isPivotVertical()) {
      runShooter(speed);
    } 
  }

  public void shoot(double speed, PivotSubsystem pivotSubsystem) {
    runShooter(speed);
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

    SmartDashboard.putNumber("shooter speed", motorLeft.getEncoder().getVelocity());
  }

  public boolean isShooterAtSpeed() {
    return motorLeft.getEncoder().getVelocity() > 4800;
  }

}
