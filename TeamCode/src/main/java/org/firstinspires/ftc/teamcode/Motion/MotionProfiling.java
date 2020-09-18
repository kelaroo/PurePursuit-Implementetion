package org.firstinspires.ftc.teamcode.Motion;

import java.util.Currency;

import static org.firstinspires.ftc.teamcode.DriveTrains.Mecanum_Roti_Negre.encoderTicksToCM;

public class MotionProfiling {
    public Point startPos;
    public Point absolutePos;
    public double absoluteAngle;

    public Point[] currPath = null;

    public MotionProfiling(Point startPos, double absoluteAngle) {
        this.startPos = new Point(startPos);
        this.absolutePos = new Point(this.startPos);
        this.absoluteAngle = absoluteAngle;
    }

    public void updatePosition(int LFticks, int LBticks, int RFticks, int RBticks) {
        double comp = 1 / Math.sqrt(2);
        double x = 0, y = 0;

        double axis1 = RFticks + LBticks;
        double axis2 = LFticks + RBticks;

        x = axis1 * comp - axis2 * comp;
        y = axis1 * comp + axis2 * comp;

        x = encoderTicksToCM(x);
        y = encoderTicksToCM(y);

        this.absolutePos.x = x;
        this.absolutePos.y = y;
    }

    public void setCurrentPath(Point[] path) {
        this.currPath = path;
    }

    /*
    * This returns an unit vector from robot position to next point of path
    * */
    public Point getVectorToNextPointOnPath(double lookDistance) {
        /*
         * currPath[i] = A
         * prevPoint = B
         * pointsOnPath[i] = P
         * */
        Point prevPoint = currPath[currPath.length-1];
        for(int i = currPath.length-2; i >= 0; i--) {
            Point[] pointsOnPath = MathFunctions.lineCircleIntersection(currPath[i], prevPoint, absolutePos, lookDistance);

            if(pointsOnPath == null)
                continue;
            if(pointsOnPath.length == 1) {
                if(MathFunctions.isPointOnSegment(currPath[i], prevPoint, pointsOnPath[0]))
                    return MathFunctions.unitVectorOfTwoPoints(absolutePos, pointsOnPath[0]);
            }
            else {
                if(MathFunctions.isPointOnSegment(currPath[i], prevPoint, pointsOnPath[0]) && MathFunctions.isPointOnSegment(currPath[i], prevPoint, pointsOnPath[1])) {
                    Point target = MathFunctions.closestPoint(pointsOnPath[0], pointsOnPath[1], prevPoint);
                    return MathFunctions.unitVectorOfTwoPoints(absolutePos, target);
                }
                else if(MathFunctions.isPointOnSegment(currPath[i], prevPoint, pointsOnPath[0])) {
                    return MathFunctions.unitVectorOfTwoPoints(absolutePos, pointsOnPath[0]);
                }
                else if(MathFunctions.isPointOnSegment(currPath[i], prevPoint, pointsOnPath[1])) {
                    return MathFunctions.unitVectorOfTwoPoints(absolutePos, pointsOnPath[1]);
                }
            }
        }
        return null;
    }
}
