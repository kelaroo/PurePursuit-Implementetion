package org.firstinspires.ftc.teamcode.Motion;

public class MathFunctions {
    public static Point[] lineCircleIntersection(Point firstPoint, Point secondPoint, Point circleCenter, double circleRadius) {
        // y = mx + n
        double m = (secondPoint.y-firstPoint.y) / (secondPoint.x-firstPoint.x);
        double n = secondPoint.y - m*secondPoint.x;

        // a*x^2 + b*x + c = 0
        double a = 1 + m*m;
        double b = -2*circleCenter.x + 2*m*n - 2*circleCenter.y*m;
        double c = circleCenter.x*circleCenter.x + n*n - 2*circleCenter.y*n + circleCenter.y*circleCenter.y - circleRadius;
        double delta = b*b - 4*a*c;

        if(delta == 0) {
            double x = -b / (2*a);
            double y = m*x + n;
            return new Point[]{new Point(x, y)};
        }
        else if(delta > 0) {
            double x1 = (-b - Math.sqrt(delta)) / (2*a);
            double y1 = m*x1 + n;

            double x2 = (-b + Math.sqrt(delta)) / (2*a);
            double y2 = m*x2 + n;

            return new Point[]{new Point(x1, y1), new Point(x2, y2)};
        }
        return null; // delta is neagative => no intersection
    }

    public static boolean isPointOnSegment(Point A, Point B, Point P) {
        double AB = Math.hypot(B.x - A.x, B.y - A.y);
        double AP = Math.hypot(P.x - A.x, P.y - A.y);
        double BP = Math.hypot(P.x - B.x, P.y - B.y);

        return AP+BP == AB;
    }

    /*
    * returns closest point O (A or B)
    * */
    public static Point closestPoint(Point A, Point B, Point O) {
        return Math.hypot(O.x-A.x, O.y-A.y) <= Math.hypot(O.x-B.x, O.y-B.y) ?
                A: B;
    }

    public static Point unitVectorOfTwoPoints(Point A, Point B) {
        double angle = Math.atan2(B.x - A.x, B.y-A.y);
        return new Point(Math.cos(angle), Math.sin(angle));
    }
}
