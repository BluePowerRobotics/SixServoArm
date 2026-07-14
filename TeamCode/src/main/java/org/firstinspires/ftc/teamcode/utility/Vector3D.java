package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

public class Vector3D {
    private double x;

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    private double y;

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    private double z;
    public double getZ() {
        return z;
    }
    public void setZ(double z) {
        this.z = z;
    }
    public double getDistance() {
        return Math.sqrt(x * x + y * y + z * z);
    }
    public double getAzimuth() {
        return Math.atan2(y, x);
    }
    public double getPolar() {
        return (getDistance() > zeroTolerance) ? Math.acos(z / getDistance()) : 0;
    }

    public void add(Vector3D other) {
        this.x += other.x;
        this.y += other.y;
        this.z += other.z;
    }

    public void clamp(@NonNull Vector3D min, @NonNull Vector3D max) {
        this.x = Math.max(min.x, Math.min(max.x, this.x));
        this.y = Math.max(min.y, Math.min(max.y, this.y));
        this.z = Math.max(min.z, Math.min(max.z, this.z));
    }

    public void clamp(double maxDistance, @NonNull Vector3D p) {
        double dist = distance(this, p);
        if (dist > maxDistance) {
            Vector3D direction = translate(this, centralSymmetry(p));
            direction = normalize(direction);
            this.x = p.x + direction.x * maxDistance;
            this.y = p.y + direction.y * maxDistance;
            this.z = p.z + direction.z * maxDistance;
        }
    }

    boolean isZero() {
        return Math.abs(x) < zeroTolerance &&
                Math.abs(y) < zeroTolerance &&
                Math.abs(z) < zeroTolerance;
    }
    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public Vector3D(Vector3D other) {
        this(other.x, other.y, other.z);
    }

    @Override
    public String toString() {
        return "( " + x + " , " + y + " , " + z + " )";
    }

    public static Vector3D ZERO = new Vector3D(0, 0, 0);
    public static double zeroTolerance = 1e-10;

    public static boolean equal(Vector3D p1, Vector3D p2) {
        return Math.abs(p1.x - p2.x) < zeroTolerance &&
                Math.abs(p1.y - p2.y) < zeroTolerance &&
                Math.abs(p1.z - p2.z) < zeroTolerance;
    }

    public static double distance(Vector3D p1, Vector3D p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) +
                Math.pow(p1.y - p2.y, 2) +
                Math.pow(p1.z - p2.z, 2));
    }

    public static double dot(Vector3D p1, Vector3D p2) {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    }

    public static Vector3D cross(Vector3D p1, Vector3D p2) {
        return new Vector3D(
                p1.y * p2.z - p1.z * p2.y,
                p1.z * p2.x - p1.x * p2.z,
                p1.x * p2.y - p1.y * p2.x
        );
    }

    public static double magnitude(Vector3D p) {
        return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    public static Vector3D normalize(Vector3D p) {
        double mag = magnitude(p);
        if (mag < zeroTolerance) return ZERO;
        return new Vector3D(p.x / mag, p.y / mag, p.z / mag);
    }

    public static Vector3D translate(Vector3D p, Vector3D offset) {
        return new Vector3D(p.x + offset.x, p.y + offset.y, p.z + offset.z);
    }

    public static Vector3D translateXYZ(Vector3D p, double dx, double dy, double dz) {
        return new Vector3D(p.x + dx, p.y + dy, p.z + dz);
    }

    public static Vector3D rotateX(Vector3D p, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Vector3D(
                p.x,
                p.y * cosA - p.z * sinA,
                p.y * sinA + p.z * cosA
        );
    }

    public static Vector3D rotateY(Vector3D p, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Vector3D(
                p.x * cosA + p.z * sinA,
                p.y,
                -p.x * sinA + p.z * cosA
        );
    }

    public static Vector3D rotateZ(Vector3D p, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Vector3D(
                p.x * cosA - p.y * sinA,
                p.x * sinA + p.y * cosA,
                p.z
        );
    }

    public static Vector3D rotateAroundAxis(Vector3D p, Vector3D axis, double angle) {
        Vector3D unitAxis = normalize(axis);
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);

        Vector3D cross = cross(unitAxis, p);
        double dot = dot(unitAxis, p);

        return new Vector3D(
                p.x * cosA + cross.x * sinA + unitAxis.x * dot * (1 - cosA),
                p.y * cosA + cross.y * sinA + unitAxis.y * dot * (1 - cosA),
                p.z * cosA + cross.z * sinA + unitAxis.z * dot * (1 - cosA)
        );
    }

    public static Vector3D midpoint(Vector3D p1, Vector3D p2) {
        return new Vector3D(
                (p1.x + p2.x) / 2,
                (p1.y + p2.y) / 2,
                (p1.z + p2.z) / 2
        );
    }

    public static Vector3D scale(Vector3D p, double factor) {
        return new Vector3D(p.x * factor, p.y * factor, p.z * factor);
    }

    public static Vector3D scale(Vector3D p, double factor, Vector3D center) {
        Vector3D translated = translateXYZ(p, -center.x, -center.y, -center.z);
        Vector3D scaled = scale(translated, factor);
        return translateXYZ(scaled, center.x, center.y, center.z);
    }

    public static Vector3D fromSpherical(double azimuth, double polar, double distance) {
        double sinPolar = Math.sin(polar);
        return new Vector3D(
                distance * sinPolar * Math.cos(azimuth),
                distance * sinPolar * Math.sin(azimuth),
                distance * Math.cos(polar)
        );
    }

    public static Vector3D centralSymmetry(Vector3D p, Vector3D center) {
        return new Vector3D(
                2 * center.x - p.x,
                2 * center.y - p.y,
                2 * center.z - p.z
        );
    }

    public static Vector3D centralSymmetry(Vector3D p) {
        return new Vector3D(-p.x, -p.y, -p.z);
    }

    public static Vector3D symmetryAboutXYPlane(Vector3D p) {
        return new Vector3D(p.x, p.y, -p.z);
    }

    public static Vector3D symmetryAboutYZPlane(Vector3D p) {
        return new Vector3D(-p.x, p.y, p.z);
    }

    public static Vector3D symmetryAboutXZPlane(Vector3D p) {
        return new Vector3D(p.x, -p.y, p.z);
    }

    public static double distanceToPlane(Vector3D point, Vector3D planeNormal, Vector3D planePoint) {
        Vector3D diff = translate(point, centralSymmetry(planePoint));
        return Math.abs(dot(diff, planeNormal));
    }

    public static Vector3D projectToPlane(Vector3D point, Vector3D planeNormal, Vector3D planePoint) {
        Vector3D diff = translate(point, centralSymmetry(planePoint));
        double distance = dot(diff, planeNormal);
        return translate(point, scale(planeNormal, -distance));
    }
    public static Vector3D symmetryAboutPlane(Vector3D point, Vector3D planeNormal, Vector3D planePoint) {
        Vector3D projection = projectToPlane(point, planeNormal, planePoint);
        return centralSymmetry(projection, point);
    }
    public static Vector2D toPoint2D(Vector3D p, Vector3D planeNormal, Vector3D planePoint) {
        Vector3D projected = projectToPlane(p, planeNormal, planePoint);
        return new Vector2D(projected.x, projected.y);
    }
    public static Vector3D calculatePlaneNormal(Vector3D p1, Vector3D p2, Vector3D p3) {
        Vector3D v1 = translate(p2, centralSymmetry(p1));
        Vector3D v2 = translate(p3, centralSymmetry(p1));
        return normalize(cross(v1, v2));
    }
    public static Vector3D linePlaneIntersection(Vector3D p, Vector3D planeNormal, Vector3D planePoint) {
        if (p.isZero()) return null;

        double d = -dot(planeNormal, planePoint);
        double denominator = dot(planeNormal, p);

        if (Math.abs(denominator) < 1e-9) {
            return null;
        }

        double t = -d / denominator;

        return scale(p, t);
    }
}