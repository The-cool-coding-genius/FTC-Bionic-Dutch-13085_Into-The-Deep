package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Odometry.GoBildaPinpointDriver;

public class AutoLibBetter {

    // PID controller variables
    private static double Kp;
    private static double Ki;
    private static double Kd;

    private static Pose target; // desired position
    private static Pose currentPosition; // current position from the Pinpoint
    private static boolean targetIsNotReached;

    private static double xError, yError, angleError; // errors in position and angle
    private static double xDerivative, yDerivative, angleDerivative; // rate of change of error
    private static double xIntegralSum, yIntegralSum, angleIntegralSum; // accumulated error over time
    private static double lastXError, lastYError, lastAngleError; // previous error for derivative calculation
    private static double xPower, yPower, anglePower;

    private double xPower, yPower, anglePower; // output power to the motor

    // Assume armMotor is an instance of a motor class with getPosition() and setPower() methods
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private static GoBildaPinpointDriver Odometry;

    //         [frontLeft, backLeft, frontRight, backRight]                      [x encoder, y encoder]
    void init(String[] driveNames, boolean[] driveReversed, String odometryName, boolean[] odoReversed) {
        frontLeft = hardwareMap.get(DcMotorEx.class, driveNames[0]);
        backLeft = hardwareMap.get(DcMotorEx.class, driveNames[1]);
        frontRight = hardwareMap.get(DcMotorEx.class, driveNames[2]);
        backRight = hardwareMap.get(DcMotorEx.class, driveNames[3]);

        if (driveReversed[0]) {
            frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        }

        if (driveReversed[1]) {
            backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        }

        if (driveReversed[2]) {
            frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        }

        if (driveReversed[3]) {
            backRight.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            backRight.setDirection(DcMotorEx.Direction.FORWARD);
        }

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

        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, odometryName);
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //Future customization for encoder directions
        Odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //odo.recalibrateIMU();
        Odometry.resetPosAndIMU();
    }

    static void changePIDValues(double p, double i, double d) {
        // Update PID coefficients
        Kp = p;
        Ki = i;
        Kd = d;
    }

    static void checkTargetReached(Pose target, double positionTolerance, double angleTolerance) {
        if (Math.abs(Odometry.getX() - targetPosition.X) >= positionTolerance) {
            if (Math.abs(Odometry.getY() - targetPosition.Y) >= positionTolerance) {
                if (Math.abs(Odometry.getAngle() - targetPosition.Angle) >= angleTolerance) {
                    targetIsNotReached = true;
                } else {
                    targetIsNotReached = false;
                }
            } else {
                targetIsNotReached = false;
            }
        } else {
            targetIsNotReached = false;
        }

     //                                                                       Degrees
    static void runToPosition (Pose target, double positionTolerance, double angleTolerance) {
        target = new Pose(0,0,0);

        xIntegralSum = 0;
        yIntegralSum = 0;
        angleIntegralSum = 0;

        lastXError = 0;
        lastYError = 0;
        lastAngleError = 0;

        checkTargetReached(target, positionTolerance, angleTolerance);

        // Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (targetIsNotReached) {

            // obtain the encoder position
            currentPosition = new Pose(Odometry.getX(), Odometry.getY(), Odometry.getAngle());
            // calculate the error
            xError = target.X - currentPosition.X;
            yError = target.Y - currentPosition.Y;
            angleError = target.Angle - currentPosition.Angle;

            // rate of change of the error
            xDerivative = (xError - lastXError) / timer.seconds();
            yDerivative = (yError - lastYError) / timer.seconds();
            angleDerivative = (angleError - lastAngleError) / timer.seconds();

            // sum of all error over time
            xIntegralSum = xIntegralSum + (xError * timer.seconds());
            yIntegralSum = yIntegralSum + (yError * timer.seconds());
            angleIntegralSum = angleIntegralSum + (angleError * timer.seconds());

            xPower = (Kp * xError) + (Ki * xIntegralSum) + (Kd * xDerivative);
            yPower = (Kp * yError) + (Ki * yIntegralSum) + (Kd * yDerivative);
            anglePower = (Kp * angleError) + (Ki * angleIntegralSum) + (Kd * angleDerivative);

            // set the power to the motors
            frontLeftMotor.setPower(yPower + xPower + anglePower);
            backLeftMotor.setPower(yPower - xPower + anglePower);
            frontRightMotor.setPower(yPower - xPower - anglePower);
            backRightMotor.setPower(yPower + xPower - anglePower);

            lastXError = xError;
            lastYError = yError;
            lastAngleError = angleError;

            // reset the timer for next time
            timer.reset();

            checkTargetReached(target, positionTolerance, angleTolerance);
        }

    }
}

static class Pose {
    double X;
    double Y;
    double Angle;

    Pose(double x, double y, double angle) {
        this.X = x;
        this.Y = y;
        this.Angle = angle;
    }
}
