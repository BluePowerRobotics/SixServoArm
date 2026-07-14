package org.firstinspires.ftc.teamcode.utility;

public class Vector2D {
    private double x;
    public double getX(){
        return x;
    }
    public void setX(double newX){
        x=newX;
    }
    private double y;
    public double getY(){
        return y;
    }
    public void setY(double newY){
        y=newY;
    }


    public double getRadian(){
        return Math.atan2(y, x);
    }
    public double getDistance(){
        return Math.hypot(x,y);
    }

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Vector2D(Vector2D p){
        this(p.x,p.y);
    }

    @Override
    public String toString() {
        return "( " + x +
                " , " + y +
                " )";
    }

    public static final Vector2D ZERO = new Vector2D(0, 0);
    public static double zeroTolerance = 1e-10;
    public static boolean equal(Vector2D p1, Vector2D p2) {
        return Math.abs(p1.x - p2.x) < zeroTolerance && Math.abs(p1.y - p2.y) < zeroTolerance;
    }
    public static double getDistance(Vector2D p1, Vector2D p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    public static Vector2D translate(Vector2D p, Vector2D offset) {
        return translate(p,offset.x,offset.y);
    }
    public static Vector2D translate(Vector2D p, double dx, double dy) {
        return new Vector2D(p.x + dx, p.y + dy);
    }
    public static Vector2D translateRD(Vector2D p, double Radian, double Distance) {
        return new Vector2D(p.x + Distance * Math.cos(Radian), p.y + Distance * Math.sin(Radian));
    }
    public static Vector2D rotate(Vector2D p, double Radian) {
        return new Vector2D(p.x * Math.cos(Radian) - p.y * Math.sin(Radian), p.x * Math.sin(Radian) + p.y * Math.cos(Radian));
    }
    public static Vector2D rotate(Vector2D p, double radian, Vector2D center) {
        Vector2D translated = translate(p, -center.x, -center.y);
        Vector2D rotated = rotate(translated, radian);
        return translate(rotated, center.x, center.y);
    }
    public static Vector2D getMidpoint(Vector2D p1, Vector2D p2) {
        return getMidpoint(p1.x,p1.y,p2.x,p2.y);
    }
    public static Vector2D getMidpoint(double x1,double y1,double x2,double y2) {
        return new Vector2D((x1 + x2) / 2, (y1 + y2) / 2);
    }
    public static Vector2D scale(Vector2D p, double factor) {
        return new Vector2D(p.x * factor, p.y * factor);
    }
    public static Vector2D scale(Vector2D p, double factor, Vector2D center) {
        Vector2D translated = translate(p, -center.x, -center.y);
        Vector2D scaled = scale(translated, factor);
        return translate(scaled, center.x, center.y);
    }
    public static Vector2D fromPolar(double radian, double distance) {
        return new Vector2D(distance * Math.cos(radian), distance * Math.sin(radian));
    }
    public static Vector2D getCentralSymmetry(Vector2D p, Vector2D center) {
        return new Vector2D(2 * center.x - p.x, 2 * center.y - p.y);
    }
    public static Vector2D getCentralSymmetry(Vector2D p) {
        return new Vector2D(-p.x, -p.y);
    }
    public static Vector2D getAxisSymmetry(Vector2D p, double k, double b) {
        if (Double.isInfinite(k)) {
            return getAxisSymmetryVertical(p, b);
        }
        if (Math.abs(k) < zeroTolerance) {
            return getAxisSymmetryHorizontal(p, b);
        }
        return getAxisSymmetrySlant(p, k, b);
    }

    private static Vector2D getAxisSymmetryVertical(Vector2D p, double x) {
        return new Vector2D(2 * x - p.x, p.y);
    }

    private static Vector2D getAxisSymmetryHorizontal(Vector2D p, double y) {
        return new Vector2D(p.x, 2 * y - p.y);
    }

    private static Vector2D getAxisSymmetrySlant(Vector2D p, double k, double b) {
        double k2 = k * k;
        double denominator = 1 + k2;

        double x1 = ((1 - k2) * p.x + 2 * k * (p.y - b)) / denominator;
        double y1 = (2 * k * p.x + (k2 - 1) * p.y + 2 * b) / denominator;

        return new Vector2D(x1, y1);
    }
    public static double dot(Vector2D p1, Vector2D p2) {
        return p1.x * p2.x + p1.y * p2.y;
    }
    public static Vector2D cross(Vector2D p1, Vector2D p2) {
        return new Vector2D(p1.x * p2.y - p1.y * p2.x, p1.y * p2.x - p1.x * p2.y);
    }
    public static Vector3D toPoint3D(Vector2D p, Vector3D planeNormal, Vector3D planePoint) {
        Vector3D unitNormal = Vector3D.normalize(planeNormal);

        Vector3D uAxis = findUAxis(unitNormal);
        Vector3D vAxis = Vector3D.cross(unitNormal, uAxis);

        Vector3D offset = Vector3D.translateXYZ(
                Vector3D.ZERO,
                p.x * uAxis.getX() + p.y * vAxis.getX(),
                p.x * uAxis.getY() + p.y * vAxis.getY(),
                p.x * uAxis.getZ() + p.y * vAxis.getZ()
        );

        return Vector3D.translate(planePoint, offset);
    }

    private static Vector3D findUAxis(Vector3D normal) {
        Vector3D reference = new Vector3D(1, 0, 0);

        if (Math.abs(Vector3D.dot(normal, reference)) > 0.9) {
            reference = new Vector3D(0, 1, 0);
        }

        double dotProduct = Vector3D.dot(reference, normal);
        return Vector3D.normalize(
                new Vector3D(
                        reference.getX() - dotProduct * normal.getX(),
                        reference.getY() - dotProduct * normal.getY(),
                        reference.getZ() - dotProduct * normal.getZ()
                )
        );
    }

    public static double hypot(double a, double b) {
        double r;
        if (Math.abs(a) > Math.abs(b)) {
            r = b/a;
            r = Math.abs(a)*Math.sqrt(1+r*r);
        } else if (b != 0) {
            r = a/b;
            r = Math.abs(b)*Math.sqrt(1+r*r);
        } else {
            r = 0.0;
        }
        return r;
    }
    public static double hypot(Vector2D... numbers){
        double opr=0;
        for (Vector2D number : numbers) {
            opr = Math.hypot(opr,number.getDistance());
        }
        return opr;
    }

    public static double normalizeAngle(double angle) {
        if(Double.isNaN(angle)){
            return Double.NaN;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return fromPolar(angle,1).getRadian();
    }
}