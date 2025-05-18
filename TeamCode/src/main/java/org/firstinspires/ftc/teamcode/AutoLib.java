package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Odometry Library")
@Disabled

public class Odometry {
  // Constants
  public final double pi = 3.14159265335897932384626433832795028841971693993;
  public final double ENCODER_WHEEL_DIAMETER = 32; // Diameter of your deadwheel
  private final double ENCODER_TICKS_PER_REVOLUTION = 2000; // Number of ticks for a revolution of the deadwheel
  private final double ENCODER_WHEEL_CIRCUMFERENCE = pi * 2.0 * (ENCODER_WHEEL_DIAMETER * 0.5);
  private final double ENCODER_WIDTH = 12.0; // Distance between parallel deadwheels

  // Variables
  private double xPos, yPos, angle;
  private double lastLeftEnc, lastRightEnc, lastNormalEnc;

  public Odometry(double xPos, double yPos, double angle) {
    this.xPos = xPos;
    this.yPos = yPos;
    this.angle = angle;
  }

  public Odometry(double angle) {
    this.xPos = 0;
    this.yPos = 0;
    this.angle = angle;
  }

  public Odometry() {
    this(0);
  }

  // https://github.com/Beta8397/virtual_robot/blob/master/TeamCode/src/org/firstinspires/ftc/teamcode/EncBot.java
  public void updatePosition(double l, double r, double n, double ang) {
    double dR = r - lastRightEnc;
    double dL = l - lastLeftEnc;
    double dN = n - lastNormalEnc;

    double rightDist = dR * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
    double leftDist = -dL * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
    double dyR = 0.5 * (rightDist + leftDist);
    double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;
    double dxR = -dN * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
    double avgHeadingRadians = Math.toRadians(angle) + headingChangeRadians / 2.0;
    double cos = Math.cos(avgHeadingRadians);
    double sin = Math.sin(avgHeadingRadians);

    xPos += dxR * sin + dyR * cos;
    yPos += -dxR * cos + dyR * sin;
    angle = normalizeAngle(ang);
    lastNormalEnc = n;
    lastLeftEnc = l;
    lastRightEnc = r;
  }

    public double normalizeAngle(double rawAngle) {
        double scaledAngle = rawAngle % 360;
        if (scaledAngle < 0) {
            scaledAngle += 360;
        }

        if (scaledAngle > 180) {
            scaledAngle -= 360;
        }

        return scaledAngle;
    }

    public String displayPositions() {
        return outStr;
    }

    public double getAngle() {
        return angle;
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public void resetOdometry() {
        resetOdometry(0, 0, 0);
    }

    public void resetOdometry(Point p) {
        resetOdometry(p.xP, p.yP, p.ang);
    }

    public void resetOdometry(double xPos, double yPos, double angle) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.angle = angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    private String format(double num) {
        return String.format("%.3f", num);
    }

    static double inchesToTicks (double inches) {
        return inches * 19.8943678865;
    }


    public void moveToPosition(double targetX, double targetY, double targetAngle, double MOE, double angleMOE, double maxTime, Pose startPose){
        ElapsedTime timer = new ElapsedTime();
        double xError = 200, yError = 200, angleError = 200; //Set arbitrary values that are larger than the MOE
        double previousTime = 0;
        while((timer.milliseconds() <= maxTime) && (Math.abs(xError) >= MOE || Math.abs(yError) >= MOE || Math.abs(angleError) >= angleMOE)){
            odometry.updatePosition();
            for (LynxModule hub : allHubs) {
              hub.clearBulkCache();
            }
            
            double k_P = 0.05; //Tune this coefficient
            double currentTime = timer.milliseconds();
            
            double xError = targetX - (odometry.getX() + startPose.x);
            double yError = targetY - (odometry.getY() + startPose.y);
            // normalize the angle to ensure efficient turning
            double angleError = normalizeAngle(targetAngle - (odometry.getAngle() + startPose.heading));
            
            double xP = k_P * xError;
            double yP = k_P * yError;
            double thetaP = k_P * angleError;
            
            double xD = calculateD(xError, previousX, currentTime, previousTime);
            double yD = calculateD(yError, previousY, currentTime, previousTime);
            double thetaD = calculateD(angleError, previousTheta, currentTime, previousTime);
            
            double xPow = xP + xD;
            double yPow = yP + yD;
            double anglePow = thetaP + thetaD;
            
            //driveFieldCentric(drive, strafe, turn);
            driveFieldCentric(yPow, xPow, anglePow);
            //experiment to see if y is your drive or strafe
            
            previousX = xError;
            previousY = yError;
            previousTheta = angleError;
            previousTime = currentTime;
    }
    }
 
 public double calculateD(double curr_error, double prev_error, double curr_time, double prev_time){

     double k_d = 0.05; //Tune this coefficient
     double error_diff = curr_error - prev_error;
     double time_diff = curr_time - prev_time;
     
     double errorByTime = error_diff / time_diff;
     double d = k_d * errorByTime;
     
     return d;
 }

  public double normalizeAngle(double rawAngle) {
    double scaledAngle = rawAngle % 360;
    if (scaledAngle < 0) {
      scaledAngle += 360;
    }

    if (scaledAngle > 180) {
      scaledAngle -= 360;
    }

    return scaledAngle;
  }
}

static class Pose {
    //1 = 1/10 inch
    double x;
    double y;
    //Degrees
    double heading;

    Pose (x, y, heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
