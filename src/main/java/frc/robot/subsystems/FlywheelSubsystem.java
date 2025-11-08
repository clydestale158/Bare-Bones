package frc.robot.subsystems;

import static frc.robot.Constants.HardwareConstants.FLYWHEEL_MOTOR_ID;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase{
    /*Variables */
     /*Motors */ //Can use SparkMax motor instead for rev devices ran with a sparkmax
    private TalonFX flywheelMotor = new TalonFX(FLYWHEEL_MOTOR_ID);
    //private SparkMax flywheelMotor = new SparkMax(FLYWHEEL_MOTOR_ID, MotorType.kBrushless); 

    //set speed, only used for logging
    private double speed;

    /* Constructor */
    public FlywheelSubsystem(){
        //add motor configs here
    }

    /*Methods */
    /**Sets the flywheel to the inputted speed, will not stop */
    public Command setSpeed(double newSpeed){
        return runOnce(()->{
            flywheelMotor.set(newSpeed);
            speed = newSpeed;
        });
    }

    /**Runs the flywheel at inputted speed and stops once ended */
    public Command runFlywheel(double speed){
        return runEnd(()->{
            flywheelMotor.set(speed);
            this.speed = speed;
        }, ()->{
            flywheelMotor.set(0);
            this.speed = 0;  
        });
    }

    @Override
    public void periodic(){
        //add dashboard logging
        SmartDashboard.putNumber("Flywheel Set Speed", speed);
    }
    
}