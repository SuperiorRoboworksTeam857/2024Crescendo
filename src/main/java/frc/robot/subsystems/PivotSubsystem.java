package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

  CANSparkMax leftMotor = new CANSparkMax(PivotConstants.pivotMotorLeftID, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(PivotConstants.pivotMotorRightID, MotorType.kBrushless);

  private final AbsoluteEncoder m_absoluteEncoder;

  private static final double horizontalAngle = 0.662;
  private static final double verticalAngle = 0.416;

  public enum Positions {
    HORIZONTAL,
    VERTICAL
  }

  private static double deltaTime = 0.02;
  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(1, 40);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(5, 0.0, 0.0, m_constraints, deltaTime);

  private double m_goalAngle = horizontalAngle;

  public PivotSubsystem() {
    m_absoluteEncoder = leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    double encoderPositionFactor = (2 * Math.PI); // radians
    double encoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    leftMotor.getEncoder().setPositionConversionFactor(encoderPositionFactor);
    leftMotor.getEncoder().setVelocityConversionFactor(encoderVelocityFactor);

    rightMotor.getEncoder().setPositionConversionFactor(encoderPositionFactor);
    rightMotor.getEncoder().setVelocityConversionFactor(encoderVelocityFactor);
  }

  @Override
  public void periodic() {
    m_controller.setGoal(m_goalAngle);
    leftMotor.set(m_controller.calculate(m_absoluteEncoder.getPosition()));
    rightMotor.set(m_controller.calculate(m_absoluteEncoder.getPosition()));


    SmartDashboard.putNumber("pivot position", m_absoluteEncoder.getPosition());
    SmartDashboard.putNumber("pivot speed", m_absoluteEncoder.getVelocity());
  }


  public void goToAngle(Positions position) {
    switch (position) {
      case HORIZONTAL:
        m_goalAngle = horizontalAngle;
        break;
      case VERTICAL:
        m_goalAngle = verticalAngle;
        break;
    }
  }

  public boolean isPivotAtGoal() {
    return Math.abs(m_absoluteEncoder.getPosition() - m_goalAngle) < 0.03;
  }

  public void resetEncoders() {
    leftMotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }
}
