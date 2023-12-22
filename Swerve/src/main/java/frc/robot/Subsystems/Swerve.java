package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.Vector2d;

public class Swerve extends SubsystemBase implements Consts{
    
    private static Swerve m_instance;
    private SwerveModule[] m_swerveModules;
    private AHRS m_gyro;
    private double m_rotationSpeed;
    private double m_targetHeadingAngle;
    private PIDController m_headingController;

    private Swerve(){
        //create swerve modules instances
        m_swerveModules[0] = new SwerveModule(MotorValues.TOP_LEFT_DRIVE_ID, MotorValues.TOP_LEFT_STEERING_ID);
        m_swerveModules[1] = new SwerveModule(MotorValues.TOP_RIGHT_DRIVE_ID, MotorValues.TOP_RIGHT_STEERING_ID);
        m_swerveModules[2] = new SwerveModule(MotorValues.DOWN_LEFT_DRIVE_ID, MotorValues.DOWN_LEFT_STEERING_ID);
        m_swerveModules[3] = new SwerveModule(MotorValues.DOWN_RIGHT_DRIVE_ID, MotorValues.DOWN_RIGHT_STEERING_ID);
        //create navx gyro instance
        m_gyro = new AHRS(SerialPort.Port.kMXP);
        //create PIDController instance
        m_headingController = new PIDController(PIDValues.HEADING_KP, PIDValues.HEADING_KI, PIDValues.HEADING_KD);
        m_headingController.setTolerance(PIDValues.HEADING_TOLERANCE);
        //init values to 0
        m_rotationSpeed = 0;
        m_targetHeadingAngle = 0;
    }

    public static Swerve getInstance(){
        if(m_instance == null)
            m_instance = new Swerve();
        return m_instance;
    }

    @Override
    public void periodic(){
        //pid on robot's heading angle
        //get current angle
        double currentAngle = m_gyro.getAngle();
        //calculate optimized angle
        double closestAngle = Funcs.closestAngle(currentAngle, m_targetHeadingAngle);
        double optimizedAngle = currentAngle + closestAngle;
        //get current pid output
        m_rotationSpeed = m_headingController.calculate(currentAngle, optimizedAngle);
    }

    public void drive(Vector2d driveVector, boolean isFieldOriented){
        //if drive values are 0 stop moving
        if(driveVector.mag() == 0 && m_rotationSpeed == 0){
            for(int i = 0; i < m_swerveModules.length; i++){
                m_swerveModules[i].setVelocity(0);
            }
        }
        
        //convert driveVector to field oriented
        if(isFieldOriented){
            driveVector.rotate(Math.toRadians(m_gyro.getYaw()));
        }
        
        Vector2d[] rotVecs = new Vector2d[m_swerveModules.length];
        for(int i = 0; i < rotVecs.length; i++){
            rotVecs[i] = new Vector2d(ChassisValues.MODULES_POSITION_VECTORS[i]);
            rotVecs[i].rotate(Math.toRadians(90));
            //change magnitude of rot vector to m_rotationSpeed
            rotVecs[i].normalise();
            rotVecs[i].mul(m_rotationSpeed);
        }

        Vector2d[] sumVectors = new Vector2d[m_swerveModules.length];
        for(int i = 0; i < sumVectors.length; i++){
            //sum rot and drive vectors 
            sumVectors[i] = new Vector2d(driveVector);
            sumVectors[i].add(rotVecs[i]);
            //make sure that the max magnitude is the max speed
            if(sumVectors[i].mag() > SpeedValues.MAX_SPEED){
                sumVectors[i].normalise();
                sumVectors[i].mul(SpeedValues.MAX_SPEED);
            }
            //set module state             
            m_swerveModules[i].setState(sumVectors[i]);
        }
    }

}
