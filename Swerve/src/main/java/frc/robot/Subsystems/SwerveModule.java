package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.Vector2d;


public class SwerveModule extends SubsystemBase implements Consts{
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steeringMotor;
    
    
    public SwerveModule(int driveMotorId, int steeringMotorId) {
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        m_steeringMotor = new CANSparkMax(steeringMotorId, MotorType.kBrushless);

        //restore the deafult values of motor so nothing would interrupt motor
        m_steeringMotor.restoreFactoryDefaults();
        m_driveMotor.restoreFactoryDefaults();
        
        //set idle mode of rotation motor
        m_steeringMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        
        //config angle pid values
        m_steeringMotor.getPIDController().setP(PIDValues.WHEEL_ANGLE_KP);
        m_steeringMotor.getPIDController().setI(PIDValues.WHEEL_ANGLE_KI);
        m_steeringMotor.getPIDController().setD(PIDValues.WHEEL_ANGLE_KD);

        //config velocity pid values
        m_driveMotor.getPIDController().setP(PIDValues.WHEEL_VELOCITY_KP);
        m_driveMotor.getPIDController().setI(PIDValues.WHEEL_VELOCITY_KI);
        m_driveMotor.getPIDController().setD(PIDValues.WHEEL_VELOCITY_KD);
        m_driveMotor.getPIDController().setFF(PIDValues.WHEEL_VELOCITY_KF);

        //convert rotations to degrees
        m_steeringMotor.getEncoder().setPositionConversionFactor(MotorValues.STEERING_GEAR_RATIO * 360);
        //convert rpm to m/s
        m_driveMotor.getEncoder().setVelocityConversionFactor(MotorValues.DRIVE_GEAR_RATIO * ChassisValues.WHEEL_PERIMETER / 60.0);
        //convert rotations to meters
        m_driveMotor.getEncoder().setPositionConversionFactor(MotorValues.DRIVE_GEAR_RATIO * ChassisValues.WHEEL_PERIMETER);
    }   

    /* 
    * @param desiredState - 2d vector - magnitude represents the target speed(in m/s)
    *                                   angle represents the target angle 
    */
   public void setState(Vector2d desiredState) {
      //get polar values from desired state vector
      double targetSpeed = desiredState.mag(); //get target speed
      double targetAngle = Math.toDegrees(desiredState.theta()); //convert target angle from radians to degrees
      double currentAngle = getAngle();
      //calculate optimal delta angle
      double optimizedFlippedDeltaTargetAngle = Funcs.closestAngle(currentAngle, targetAngle - 180);
      double optimizedNormalDeltaTargetAngle = Funcs.closestAngle(currentAngle, targetAngle);

      double optimizedDeltaTargetAngle = 0;
      if(Math.abs(optimizedNormalDeltaTargetAngle) > Math.abs(optimizedFlippedDeltaTargetAngle)){
           optimizedDeltaTargetAngle = optimizedFlippedDeltaTargetAngle;
      }
      else{
           optimizedDeltaTargetAngle = optimizedNormalDeltaTargetAngle;
      }
      //turn module to target angle
      m_steeringMotor.getPIDController().setReference(currentAngle + optimizedDeltaTargetAngle, ControlType.kPosition);
      //dot product to current state
      targetSpeed *= Math.cos(Math.toRadians(optimizedNormalDeltaTargetAngle));
      //set speed of module at target speed
      m_driveMotor.getPIDController().setReference(targetSpeed, ControlType.kVelocity);
  }

  /**
   * set state of module state
   * @param magnitude - target speed
   * @param angle - target angle(in degrees)
   */
  public void setState(double magnitude, double angle){
    setState(new Vector2d(Math.cos(Math.toRadians(angle)) * magnitude, Math.sin(Math.toRadians(angle)) * magnitude));
  }

  /**
   * @return module angle in degrees
   */
  public double getAngle(){
    return m_steeringMotor.getEncoder().getPosition();
  }

  /**
   * turn module to angle
   * @param angle - target angle in degrees 
   */
  public void setAngle(double angle){
    m_steeringMotor.getPIDController().setReference(angle, ControlType.kPosition);
  }
  
  /**
   * @return module velocity in m/s 
   */
  public double getVelocity(){
    return m_driveMotor.getEncoder().getVelocity();
  }
  
  /**
   * set module velocity 
   * @param velocity - target velocity in m/s
   */
  public void setVelocity(double velocity){
    m_driveMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
  }
  
  /**
   * @return module position in meters
   */
  public double getPos(){
    return m_driveMotor.getEncoder().getPosition();
  }

  /**
   * set encoder position value
   * @param pos - new encoder position value in meters
   */
  public void setEncoderPosition(double pos){
    m_driveMotor.getEncoder().setPosition(pos);
  }

}
