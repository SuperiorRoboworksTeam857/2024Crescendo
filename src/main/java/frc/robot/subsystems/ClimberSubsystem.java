/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(ClimberConstants.leftArmID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(ClimberConstants.rightArmID, MotorType.kBrushless);
  
  RelativeEncoder leftMotorEncoder = leftMotor.getEncoder();
  RelativeEncoder rightMotorEncoder = rightMotor.getEncoder();

  public ClimberSubsystem() {}

  @Override
  public void periodic() {}

  public void runArms(double rightSpeed, double leftSpeed)
  {
    runRightArm(rightSpeed);
    runLeftArm(leftSpeed);
  }

  public void runRightArm(double speed)
  {
    //positive speed and position are up

    speed = MathUtil.applyDeadband(speed, Constants.Swerve.stickDeadband);

    boolean movingUp = (speed > 0);
    boolean movingDown = (speed < 0);

    if ((rightMotorEncoder.getPosition() > 0 && movingDown) ||
       (rightMotorEncoder.getPosition() < 530 && movingUp)) {
      rightMotor.set(speed);
    }
    else if (movingDown) {
      rightMotor.set(speed*0.1);
    }
    else {
      rightMotor.set(0);
    }
    SmartDashboard.putNumber("right motor speed", speed);
    SmartDashboard.putNumber("rightArmPosition", rightMotorEncoder.getPosition());
  }

  public void runLeftArm(double speed)
  {
    //negative speed and position are up

    speed = MathUtil.applyDeadband(speed, Constants.Swerve.stickDeadband);

    boolean movingUp = (speed < 0);
    boolean movingDown = (speed > 0);

    if ((leftMotorEncoder.getPosition() < 0 && movingDown) ||
        (leftMotorEncoder.getPosition() > -475 && movingUp)) {
      leftMotor.set(speed);
    }
    else if (movingDown) {
      leftMotor.set(speed*0.1);
    }
    else {
      leftMotor.set(0);
    }

    SmartDashboard.putNumber("left motor speed", speed);
    SmartDashboard.putNumber("leftArmPosition", leftMotorEncoder.getPosition());
  }

  

}
