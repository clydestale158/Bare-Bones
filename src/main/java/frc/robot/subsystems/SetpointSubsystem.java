package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareConstants.SETPOINT_MOTOR_ID;

public class SetpointSubsystem extends SubsystemBase {
    /* Variables */
    /* Motors */ // Can use SparkMax motor instead for rev devices ran with a sparkmax
    private TalonFX setpointMotor = new TalonFX(SETPOINT_MOTOR_ID);
    // private SparkMax setpointMotor = new SparkMax(SETPOINT_MOTOR_ID,
    // MotorType.kBrushless);

    /* PID and ffe controllers */
    private PIDController controller = new PIDController(0, 0, 0);
    private ArmFeedforward ffeController = new ArmFeedforward(2, 0.5, 0); // change type for different mechanisms, this is for pivots

    /* Position */
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private double setpoint = 0;

    /* Constructor */
    public SetpointSubsystem() {
        //add motor configs here
    }

    /* Methods */
    public Command changeSetpoint(double newSetpoint) {
        return runOnce(() -> {
            setpoint = newSetpoint;
        });
    }

    private double getEffort() {
        return ffeController.calculate(Units.degreesToRadians(encoder.get()), 0)
                + controller.calculate(encoder.get(), setpoint);
    }

    @Override
    public void periodic() {
        setpointMotor.setVoltage(getEffort());

        // logging to dashboard
        SmartDashboard.putNumber("Setpoint Mech setpoint", setpoint);
        SmartDashboard.putNumber("Setpoint Mech Motor Encoder", setpointMotor.getRotorPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Setpoint Mech Encoder",
        // setpointMotor.getEncoder().getPosition());

    }
}