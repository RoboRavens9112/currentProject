package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TankDrive;
// import edu.wpi.first.math.controller.PIDController;


public class DriveTrain extends SubsystemBase {
    private final CANSparkMax m_leftDrive1 = new CANSparkMax(10, MotorType.kBrushed);
    private final CANSparkMax m_leftDrive2 = new CANSparkMax(11, MotorType.kBrushed);
    private final CANSparkMax m_rightDrive1 = new CANSparkMax(12, MotorType.kBrushed);
    private final CANSparkMax m_rightDrive2 = new CANSparkMax(13, MotorType.kBrushed);
    //these set the motors as the type CANSparkMax to m_xdirectionDrive
    //m_xdirectionDrive is an object that you can apply CANSparkMax methods to such as .set() to move or .getEncoder() to create an encoder
    private double slewSpeed = 2.5;
    //reaches top speed in .7 seconds. you can calculate speed it will take slew rate limiter to accelerate with equation 1/x

    SlewRateLimiter rightFilter1 = new SlewRateLimiter(slewSpeed);
    SlewRateLimiter rightFilter2 = new SlewRateLimiter(slewSpeed);
    SlewRateLimiter leftFilter1 = new SlewRateLimiter(slewSpeed);
    SlewRateLimiter leftFilter2 = new SlewRateLimiter(slewSpeed);
    //PIDController pid = new PIDController(0, 0, 0);

    //this method establishes the tankdrive as the default command for the drivetrain
    //we still need to run this method in our initialization for our roboto to establish the default command and add itt to the commandscheduler
    public void initDefaultCommand() {
        setDefaultCommand(new TankDrive());
        PortForwarder.add(5800, "photonvision.local", 5800);
        System.out.println("test?");
    }

    //moves motors forward, pushes the value inputed into the () for setLeftMotorsForward() into the input for the motor speed set
    //we have the filter.calculate(speed) there so we apply the slew rate limiter to the input. this makes power transition a lot smoother
    public void setLeftMotorsForward(double speed) {
        m_leftDrive1.set(leftFilter1.calculate(speed));
        m_leftDrive2.set(leftFilter2.calculate(speed));
    }

    public void setRightMotorsForward(double speed) {
        m_rightDrive1.set(rightFilter1.calculate(speed));
        m_rightDrive2.set(rightFilter2.calculate(speed));
    }

    public void setLeftMotorsForwardOverdrive(double speed) {
        m_leftDrive1.set(speed);
        m_leftDrive2.set(speed);
    }
    
    public void setRightMotorsForwardOverdrive(double speed) {
        m_rightDrive1.set(speed);
        m_rightDrive2.set(speed);
    }

    public void stopRobot() {
        m_rightDrive1.set(0);
        m_rightDrive2.set(0);
        m_leftDrive1.set(0);
        m_leftDrive2.set(0);
    }
}