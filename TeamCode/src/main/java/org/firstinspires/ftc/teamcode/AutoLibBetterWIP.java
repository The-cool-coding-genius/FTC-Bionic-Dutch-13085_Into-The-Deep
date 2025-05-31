package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Disabled
@Autonomous (name = "Auto Library")
public class AutoLib extends LinearOpMode {

    // PID controller variables
    private static double Kp;
    private static double Ki;
    private static double Kd;
    private static Pose currentPosition; // current position from the Pinpoint
    private static boolean targetIsNotReached;
    private static Drivetrain drivetrain;
    private String[] driveNames;
    private boolean[] motorReversed;
    private DcMotorEx.ZeroPowerBehavior zeroPowerBehavior;
    private static GoBildaPinpointDriver Odometry;

    @Override
    public void runOpMode(){

        driveNames = new String[] {"frontLeft", "backLeft", "frontRight", "backRight"};
        motorReversed = new boolean[] {false, false, true, true};
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        drivetrain = new Drivetrain();
        try {
            drivetrain.init(driveNames, motorReversed, zeroPowerBehavior);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        while (opModeIsActive()) {
            currentPosition = new Pose(Odometry.getPosX(DistanceUnit.INCH), Odometry.getPosY(DistanceUnit.INCH), Odometry.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("Running...");
            telemetry.update();
        }
    }

    //         [frontLeft, backLeft, frontRight, backRight]                      [x encoder, y encoder]
    void init(@NonNull String odometryName, @NonNull GoBildaPinpointDriver.EncoderDirection[] deadwheelReversed) {

        Odometry = hardwareMap.get(GoBildaPinpointDriver.class, odometryName);
        Odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //Future customization for encoder directions
        Odometry.setEncoderDirections(deadwheelReversed[0], deadwheelReversed[1]);

        //odo.recalibrateIMU();
        Odometry.resetPosAndIMU();
    }

    static void changePIDValues(double p, double i, double d) {
        // Update PID coefficients
        Kp = p;
        Ki = i;
        Kd = d;
    }

    static void checkTargetReached(@NonNull Pose target, double positionTolerance, double angleTolerance) {
        if (Math.abs(Odometry.getPosX(DistanceUnit.INCH) - target.X) >= positionTolerance) {
            if (Math.abs(Odometry.getPosY(DistanceUnit.INCH) - target.Y) >= positionTolerance) {
                if (Math.abs(Odometry.getHeading(AngleUnit.DEGREES) - target.Heading) >= angleTolerance) {
                    targetIsNotReached = true;
                } else {
                    targetIsNotReached = false;
                    drivetrain.drive(0, 0, 0);
                }
            } else {
                targetIsNotReached = false;
                drivetrain.drive(0, 0, 0);
            }
        } else {
            targetIsNotReached = false;
            drivetrain.drive(0, 0, 0);
        }
    }

    //                                                                               Degrees
    static void runToPosition (@NonNull Pose target, double positionTolerance, double angleTolerance) {
        double xError, yError, headingError;
        double xDerivative, yDerivative, headingDerivative; // rate of change of error
        double xIntegralSum, yIntegralSum, headingIntegralSum; // accumulated error over time
        double lastXError, lastYError, lastHeadingError; // previous error for derivative calculation
        double xPower, yPower, headingPower;

        target = new Pose (Math.abs(target.X), Math.abs(target.Y), Math.abs(target.Heading));
        positionTolerance = Math.abs(positionTolerance);
        xIntegralSum = 0;
        yIntegralSum = 0;
        headingIntegralSum = 0;

        lastXError = 0;
        lastYError = 0;
        lastHeadingError = 0;

        checkTargetReached(target, positionTolerance, angleTolerance);

        // Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();
        while (targetIsNotReached) {

            // obtain the encoder position
            currentPosition = new Pose(Odometry.getPosX(DistanceUnit.INCH), Odometry.getPosY(DistanceUnit.INCH), Odometry.getHeading(AngleUnit.DEGREES));
            // calculate the error
            xError = target.X - currentPosition.X;
            yError = target.Y - currentPosition.Y;
            headingError = target.Heading - currentPosition.Heading;

            // rate of change of the error
            xDerivative = (xError - lastXError) / timer.seconds();
            yDerivative = (yError - lastYError) / timer.seconds();
            headingDerivative = (headingError - lastHeadingError) / timer.seconds();

            // sum of all error over time
            xIntegralSum = xIntegralSum + (xError * timer.seconds());
            yIntegralSum = yIntegralSum + (yError * timer.seconds());
            headingIntegralSum = headingIntegralSum + (headingError * timer.seconds());

            xPower = (Kp * xError) + (Ki * xIntegralSum) + (Kd * xDerivative);
            yPower = (Kp * yError) + (Ki * yIntegralSum) + (Kd * yDerivative);
            headingPower = (Kp * headingError) + (Ki * headingIntegralSum) + (Kd * headingDerivative);

            // set the power to the motors
            drivetrain.drive (xPower, yPower, headingPower);


            lastXError = xError;
            lastYError = yError;
            lastHeadingError = headingError;

            // reset the timer for next time
            timer.reset();

            checkTargetReached(target, positionTolerance, angleTolerance);
        }

        }
    class Drivetrain {
        DcMotorEx frontLeft, backLeft, frontRight, backRight;

        /**
         * Initializes the drivetrain motors.
         * @param motorNames  An array of 4 motor names: frontLeft, backLeft, frontRight, backRight.
         * @param motorReversed An array of 4 booleans in format [frontLeft, backLeft, frontRight, backRight] indicating if each motor is reversed.
         * @param zeroPowerBehavior The desired zero power behavior (e.g. BRAKE or FLOAT).
         */
        void init(String[] motorNames, boolean[] motorReversed, DcMotor.ZeroPowerBehavior zeroPowerBehavior) throws Exception {
            if (motorNames.length != 4) {
                throw new Exception("motorNames must have 4 parameters.");
            }
            if (motorReversed.length != 4) {
                throw new Exception("motorReversed must have 4 parameters.");
            }


            frontLeft = hardwareMap.get(DcMotorEx.class, motorNames[0]);
            backLeft = hardwareMap.get(DcMotorEx.class, motorNames[1]);
            frontRight = hardwareMap.get(DcMotorEx.class, motorNames[2]);
            backRight = hardwareMap.get(DcMotorEx.class, motorNames[3]);

            if (motorReversed[0]) {
                frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            }

            if (motorReversed[1]) {
                backLeft.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                backLeft.setDirection(DcMotorEx.Direction.FORWARD);
            }

            if (motorReversed[2]) {
                frontRight.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                frontRight.setDirection(DcMotorEx.Direction.FORWARD);
            }

            if (motorReversed[3]) {
                backRight.setDirection(DcMotorEx.Direction.REVERSE);
            } else {
                backRight.setDirection(DcMotorEx.Direction.FORWARD);
            }

            frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
            backLeft.setZeroPowerBehavior(zeroPowerBehavior);
            frontRight.setZeroPowerBehavior(zeroPowerBehavior);
            backRight.setZeroPowerBehavior(zeroPowerBehavior);
            frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        /**
         * Drives a holonomic (mecanum) four-wheel robot. This may be used in the Teleoperational period.
         * @param xPower        Robot power in the X direction. This may be your "forward" or "strafe" direction.
         * @param yPower        Robot power in the Y direction. This may be your "forward" or "strafe" direction.
         * @param headingPower  Robot rotational power in degrees.
         */
        void drive(double xPower, double yPower, double headingPower) {
            frontLeft.setPower(yPower + xPower + headingPower);
            backLeft.setPower(yPower - xPower + headingPower);
            frontRight.setPower(yPower - xPower - headingPower);
            backRight.setPower(yPower + xPower - headingPower);
        }
    }
    static class Pose {
        double X;
        double Y;
        double Heading;

        Pose(double x, double y, double heading) {
            this.X = x;
            this.Y = y;
            this.Heading = heading;
        }
    }
}
