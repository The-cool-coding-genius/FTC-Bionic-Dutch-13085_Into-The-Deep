public class AutoLibBetter extends LinearOpMode{

    // PID controller variables
    private static double Kp;
    private static double Ki;
    private static double Kd;

    private double reference; // desired position
    private double encoderPosition; // current position from the motor encoder
    private double xError, yError, angleError; // errors in position and angle
    private double derivative; // rate of change of error
    private double integralSum; // accumulated error over time
    private double lastError; // previous error for derivative calculation

    private double xPower, yPower, anglePower; // output power to the motor

    // Assume armMotor is an instance of a motor class with getPosition() and setPower() methods
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private GoBildaPinpointDriver Odometry;

    static void changePIDValues(double p, double i, double d) {
        // Update PID coefficients
        Kp = p;
        Ki = i;
        Kd = d;
    }
    static void runToPosition (double targetPosition) {
        reference = someValue;

integralSum = 0;

lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
ElapsedTime timer = new ElapsedTime();

while (setPointIsNotReached) {


    // obtain the encoder position
    encoderPosition = this.getPosition();
    // calculate the error
    error = reference - encoderPosition;

    // rate of change of the error
    derivative = (error - lastError) / timer.seconds();

    // sum of all error over time
    integralSum = integralSum + (error * timer.seconds());

    out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

    armMotor.setPower(out);

    lastError = error;

    // reset the timer for next time
    timer.reset();
    }

}

}
