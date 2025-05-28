package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Odometry.GoBildaPinpointDriver;

public class AutoLibBetter extends LinearOpMode{

    // PID controller variables
    private static double Kp;
    private static double Ki;
    private static double Kd;

    private static Pose target; // desired position
    private static double encoderPosition; // current position from the motor encoder
    private static double xError, yError, angleError; // errors in position and angle
    private static double derivative; // rate of change of error
    private static double integralSum; // accumulated error over time
    private static double lastError; // previous error for derivative calculation

    private double xPower, yPower, anglePower; // output power to the motor

    // Assume armMotor is an instance of a motor class with getPosition() and setPower() methods
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private GoBildaPinpointDriver Odometry;
    @Override
    void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    static changePIDValues(double p, double i, double d) {
        // Update PID coefficients
        Kp = p;
        Ki = i;
        Kd = d;
    }
    static void runToPosition (double targetPosition) {
        target = new Pose(0,0,0);;

        integralSum = 0;

        lastError = 0;

        // Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (setPointIsNotReached) {


            // obtain the encoder position
            encoderPosition = Pose();
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

static class Pose {
    double x;
    double y;
    double angle;

    Pose(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
}
