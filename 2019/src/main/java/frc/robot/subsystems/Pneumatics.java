package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.RunCompressor;

public class Pneumatics extends Subsystem {

    private static Pneumatics m_instance;

    public Compressor comp =  new Compressor(0);
    private final Relay compRelay = new Relay(RobotMap.COMPRESSOR);

    public Pneumatics() {

        super();
        comp.setClosedLoopControl(true);
        comp.start();
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new RunCompressor());
    }

    public static Pneumatics getInstance() {
        if (m_instance == null) {
        m_instance = new Pneumatics();
        }
        return m_instance;
    }

    public void setCompressor(boolean on) {
        if(on) {
            compRelay.set(Value.kForward);
        }
        else {
            compRelay.set(Value.kOff);
        }
    }
}