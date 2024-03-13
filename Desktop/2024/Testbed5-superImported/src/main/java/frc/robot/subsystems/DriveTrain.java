package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TankDrive;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.controller.PIDController;


public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_leftDrive1 = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax m_leftDrive2 = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax m_rightDrive1 = new CANSparkMax(18, MotorType.kBrushless);
    private final CANSparkMax m_rightDrive2 = new CANSparkMax(19, MotorType.kBrushless);
    //these set the motors as the type CANSparkMax to m_xdirectionDrive
    //m_xdirectionDrive is an object that you can apply CANSparkMax methods to such as .set() to move or .getEncoder() to create an encoder

    //PIDController pid = new PIDController(0, 0, 0);
    SlewRateLimiter rightFilter1 = new SlewRateLimiter(0.5);
    SlewRateLimiter rightFilter2 = new SlewRateLimiter(0.5);
    SlewRateLimiter leftFilter1 = new SlewRateLimiter(0.5);
    SlewRateLimiter leftFilter2 = new SlewRateLimiter(0.5);

    public void motorsInvert() {
        m_rightDrive1.setInverted(true);
        m_rightDrive2.setInverted(true);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new TankDrive());
    }
    //this method establishes the tankdrive as the default command for the drivetrain
    //we still need to run this method in our initialization for our roboto to establish the default command and add itt to the commandscheduler

    public void setLeftMotorsForward(double speed) {
        m_leftDrive1.set(leftFilter1.calculate(speed));
        m_leftDrive2.set(leftFilter2.calculate(speed));
    }
    //moves motors forward, pushes the value inputed into the () for setLeftMotorsForward() into the input for the motor speed set
    //we have the filter.calculate(speed) there so we apply the slew rate limiter to the input. this makes power transition a lot smoother
    public void setRightMotorsForward(double speed) {
        m_rightDrive1.set(rightFilter1.calculate(speed));
        m_rightDrive2.set(rightFilter2.calculate(speed));
    }

    public void setRightMotorsForwardOverdrive(double speed) {
        m_rightDrive1.set(speed);
        m_rightDrive2.set(speed);
    }

    public void setLeftMotorsForwardOverdrive(double speed) {
        m_leftDrive1.set(speed);
        m_leftDrive2.set(speed);
    }  

    public void setRightMotorsTurn(double speed) {
        m_rightDrive1.set(speed);
        m_rightDrive2.set(speed);
    }

    public void setLeftMotorsTurn(double speed) {
        m_leftDrive1.set(speed);
        m_leftDrive2.set(speed);
    }
        

}