package frc.robot.Subsystems;

import frc.robot.Commands.ElevatorCommand;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
@Logged
public class ElevatorSubsystem extends SubsystemBase{

    private final SparkMax m_elevator;
    private final RelativeEncoder m_encoder;
    private final DigitalInput m_limitSwitch;

    public ElevatorSubsystem() {

        m_elevator = new SparkMax(ElevatorConstants.kElevatorMotorPort, MotorType.kBrushless);
        m_elevator.configure(ElevatorConstants.kElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_limitSwitch = new DigitalInput(ElevatorConstants.kElevatorLimitSwitchPort);

        m_encoder = m_elevator.getEncoder();

        configureSysid();
    }

    /**
     * Automatically limits voltage to ElevatorConstants.kMaxMotorVoltage
     * @param speed Speed -1 to 1
     */
    public void setMotor(double speed) {
        if(ElevatorConstants.kClampBatteryVoltageToMaxVoltage)
        {
            m_elevator.setVoltage(Math.max(Math.min(speed, 1.0), -1.0) * ElevatorConstants.kMaxMotorVoltage);
        }
        else
        {
            m_elevator.setVoltage(speed * ElevatorConstants.kMaxMotorVoltage);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator pos", getEncoderPosition());
        getLimitSwitch();

    }

    public void resetEncoder() 
    {
        m_encoder.setPosition(0.05);
    }

    public double getEncoderPosition() 
    {    
        return m_encoder.getPosition() * ElevatorConstants.kElevatorEncoderRot2Meters;
    }

    public double getEncoderVelocity() 
    {
        return m_encoder.getVelocity() * ElevatorConstants.kElevatorEncoderRot2Meters;
    }

    public boolean getLimitSwitch()
    {
        return !m_limitSwitch.get();
    }

    public void setVoltage(Voltage voltage)
    {
        m_elevator.setVoltage(voltage);
    }

    // ======= SYSID ======= \\
    private SysIdRoutine routine;

    private Distance getLinearPosition()
    {
        return Meters.of(getEncoderPosition());
    }

    private LinearVelocity getLinearVelocity()
    {
        return MetersPerSecond.of(getEncoderVelocity());
    }

    private Voltage getAppliedVoltage()
    {
        return Volts.of(m_elevator.getAppliedOutput() * m_elevator.getBusVoltage());
    }

    private void configureSysid()
    {
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.5),
                Volts.of(2),
                Seconds.of(5)
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> 
                { 
                    setVoltage(voltage); 
                },
                (log) -> 
                {
                    log.motor("elevatormotor").
                    voltage(getAppliedVoltage()).
                    linearPosition(getLinearPosition()).
                    linearVelocity(getLinearVelocity());
                },
            this)
        );
    }

    public Command quasiForward()   { return routine.quasistatic(SysIdRoutine.Direction.kForward); }
    public Command quasiReverse()   { return routine.quasistatic(SysIdRoutine.Direction.kReverse); }
    public Command dynamicForward() { return routine.dynamic(SysIdRoutine.Direction.kForward);     }
    public Command dynamicReverse() { return routine.dynamic(SysIdRoutine.Direction.kReverse);     }
}