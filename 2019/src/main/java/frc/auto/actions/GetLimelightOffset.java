package frc.auto.actions;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Timer;

public class GetLimelightOffset implements Action {
    private static final Limelight mLL = Limelight.getInstance();
    public double limelightOffset = 0.0;
    public double counter = 0.0;
    public double averageOffset = 0.0;
    private double mStartTime;
    private final double mDuration;

    public GetLimelightOffset(double duration) {
        mDuration = duration;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration;
    }

    @Override
    public void update() {
        limelightOffset += mLL.GetOffsetAngle();
        counter++;
        averageOffset = limelightOffset / counter;
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    public double getAverageOffset() {
        return averageOffset;
    }
}