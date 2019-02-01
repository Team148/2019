package frc.auto.actions;

import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain;
import lib.physics.DriveCharacterization;
import lib.util.DriveSignal;
import lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class CollectCurvatureData implements Action {
    private static final double kMaxPower = 0.4;
    private static final double kStartPower = 0.2;
    private static final double kStartTime = 0.25;
    private static final double kRampRate = 0.02;
    private static final Drivetrain mDrive = Drivetrain.getInstance();
    private static final RobotState mRobotState = RobotState.getInstance();

    private final ReflectingCSVWriter<DriveCharacterization.CurvatureDataPoint> mCSVWriter;
    private final List<DriveCharacterization.CurvatureDataPoint> mCurvatureData;
    private final boolean mReverse;

    private boolean isFinished = false;
    private double mStartTime = 0.0;

    /**
     * @param data     reference to the list where data points should be stored
     * @param highGear use high gear or low
     * @param reverse  if true drive in reverse, if false drive normally
     */

    public CollectCurvatureData(List<DriveCharacterization.CurvatureDataPoint> data, boolean reverse) {
        mCurvatureData = data;
        mReverse = reverse;
        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/CURVATURE_DATA.csv", DriveCharacterization.CurvatureDataPoint.class);

    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(kStartPower, kStartPower));
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double t = Timer.getFPGATimestamp() - mStartTime;
        if (t < kStartTime) { //give the robot some time to accelerate before recording data
            return;
        }
        double rightPower = kStartPower + (t - kStartTime) * kRampRate;
        if (rightPower > kMaxPower) {
            isFinished = true;
            return;
        }
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kStartPower, (mReverse ? -1.0 : 1.0) * rightPower));
        mCurvatureData.add(new DriveCharacterization.CurvatureDataPoint(
                mRobotState.getPredictedVelocity().dx,
                mRobotState.getPredictedVelocity().dtheta,
                kStartPower,
                rightPower
        ));
        mCSVWriter.add(mCurvatureData.get(mCurvatureData.size() - 1));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush();
    }
}
