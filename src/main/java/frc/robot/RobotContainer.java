package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.SetpointSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.generated.TunerConstants;

public class RobotContainer {
    /* variables */
    /* Controllers */
    private CommandXboxController xbox = new CommandXboxController(0);
    private CommandXboxController xbox2 = new CommandXboxController(1);

    //specific to CTRE/pheonix generated swerve drivetrain
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

    /* Homemade object instiantiations */
    private DrivetrainSubsystem driveSubsys = TunerConstants.createDrivetrain();
    private FlywheelSubsystem flywheelSubsys = new FlywheelSubsystem();
    private SetpointSubsystem setpointSubsys = new SetpointSubsystem();

    private Superstructure superstructure = new Superstructure();

    /* Sendable choosers */
    public SendableChooser<Command> autoChooser;
    public SendableChooser<Double> forwardPolarityChooser = new SendableChooser<>();
    public SendableChooser<Double> sidewaysPolarityChooser = new SendableChooser<>();

    /** Constructor */
    public RobotContainer() {
        configButtonBindings();
        // registerNamedCommands();//ONLY for pathplanner
        // driveSubsys.configAutoBuilder();//ONLY for pathplanner
        configSendableChoosers();
    }

    private void configButtonBindings() {
        // examples
        xbox.a().onTrue(setpointSubsys.changeSetpoint(0));
        xbox.b().whileTrue(flywheelSubsys.runFlywheel(0.5));
        // add/change

        //setup drive as a default command
        //setup specifically for pheonix tuner generated drivetrain
        driveSubsys.setDefaultCommand(driveSubsys
                .applyRequest(() -> drive.withVelocityX(xbox.getLeftY() * forwardPolarityChooser.getSelected() * 0.8)
                        .withVelocityY(xbox.getLeftX() * sidewaysPolarityChooser.getSelected() * 0.8)
                        .withRotationalRate(xbox.getRightX() * 3)));
    }

    public void registerNamedCommands() {// pathplanner specific
        // example
        NamedCommands.registerCommand("Set Flywheel", flywheelSubsys.setSpeed(0.7));
    }

    public void configSendableChoosers() {
        // autoChooser = AutoBuilder.buildAutoChooser(); //FOR USE WITH PATHPLANNER ONLY
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Null", Commands.print("NO AUTON SELECTED"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        forwardPolarityChooser = new SendableChooser<>();
        forwardPolarityChooser.setDefaultOption("Standard", 1.0);
        forwardPolarityChooser.addOption("Inverted", -1.0);
        SmartDashboard.putData("Forward Polarity Chooser", forwardPolarityChooser);

        sidewaysPolarityChooser = new SendableChooser<>();
        sidewaysPolarityChooser.setDefaultOption("Standard", 1.0);
        sidewaysPolarityChooser.addOption("Inverted", -1.0);
        SmartDashboard.putData("Sideways Polarity Chooser", sidewaysPolarityChooser);
    }
}