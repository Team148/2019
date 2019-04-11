package lib.util;

/**
 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that
 * the "turning" stick controls the curvature of the robot's path rather than
 * its rate of heading change. This helps make the robot more controllable at
 * high speeds. Also handles the robot's quick turn functionality - "quick turn"
 * overrides constant-curvature turning for turn-in-place maneuvers.
 */
public class ArcadeDriveHelper {

    private static final double kThrottleDeadband = 0.02;
    // private static final double kWheelDeadband = 0.02;

    private static boolean squareInputs = true;

    public DriveSignal arcadeDrive(double xSpeed, double zRotation) {

        return arcadeDrive(xSpeed, zRotation, squareInputs);
    }
    public DriveSignal arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    
        xSpeed = limit(xSpeed);
        xSpeed = handleDeadband(xSpeed, kThrottleDeadband);
    
        zRotation = limit(zRotation);
        zRotation = handleDeadband(zRotation, kThrottleDeadband);
    
        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }
    
        double leftMotorOutput;
        double rightMotorOutput;
    
        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    
        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
            } else {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
            } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
            }
        }
        return new DriveSignal(leftMotorOutput, rightMotorOutput);
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    protected double limit(double value) {
        if (value > 1.0) {
          return 1.0;
        }
        if (value < -1.0) {
          return -1.0;
        }
        return value;
    }
}
