package org.firstinspires.ftc.teamcode;

import AutoLib.java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

static Pose[] queue;
static int i;
static Pose previousTarget;
static Pose currentTarget;
static double threshold;
static Pose currentPosition;

@Autonomous(name="Odometry Autonomous")
public class Auto extends LinearOpMode {
    public void init(){
        //1 = 1 Inches
        //Remember to use the inchesToTicks function to make this graph localization compatible with GOBilda 4-Bar odometry pods.
        //       Starting Pose         Step 1
        queue = [new Pose(9, 9, 0), new Pose(720, 720, 180)];
        threshold = 1;
        i = 1;
    }

    @Override
    public void runOpMode(){
        currentPosition = new Pose (odometry.getX() + queue[0].x, odometry.getY() + queue[0].y, odometry.getHeading() + queue[0].heading);
        //If there are targets left
        if (i < queue.length) {
            previousTarget = queue[i - 1];
            currentTarget = queue[i];
            //If X position is within threshold range of target
            if (odometry.getX() <= previousTarget.x + threshold && odometry.getX() >= previousTarget.x - threshold) {
                if (odometry.getY() <= previousTarget.y + threshold && odometry.getY() >= previousTarget.y - threshold) {
                    Odometry.moveToPosition(inchesToTicks(currentTarget.x), inchesToTicks(currentTarget.y), Math.toRadians(currentTarget.heading), 1, 2, 3000, queue[0]);
                }
            }
        }
        else {
            return;
        }
    }
}
