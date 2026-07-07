package org.firstinspires.ftc.teamcode.utility.LinearInterpolation;

/**
 * Lightweight linear interpolator for strictly increasing x data.
 *
 * <p>For a query xq in [x[i], x[i+1]], the linear interpolation is:</p>
 * <pre>
 * t = (xq - x[i]) / (x[i+1] - x[i])
 * y = y[i] + t * (y[i+1] - y[i])
 * </pre>
 *
 * <p>Queries outside the provided x-range are clamped to the nearest endpoint.
 * 该线性插值表（LinearInterpolation 类）对于落在给定数据范围外的点，采用“端点钳制”策略（clamping to endpoints）：
 * 如果查询点 xq 小于等于最小的 x（x[0]），则返回 y[0]。
 * 如果查询点 xq 大于等于最大的 x（x[n-1]），则返回 y[n-1]。
 * 即：超出范围的查询不会外推（extrapolate），而是直接返回最近端点的 y 值。</p>
 */
public final class LinearInterpolation {
    private final double[] x;
    private final double[] y;

    public LinearInterpolation(double[] x, double[] y) {
        if (x == null || y == null) {
            throw new IllegalArgumentException("x and y must be non-null.");
        }
        if (x.length != y.length) {
            throw new IllegalArgumentException("x and y must be the same length.");
        }
        if (x.length < 2) {
            throw new IllegalArgumentException("At least two points are required.");
        }

        this.x = x.clone();
        this.y = y.clone();
        for (int i = 1; i < this.x.length; i++) {
            if (this.x[i] <= this.x[i - 1]) {
                throw new IllegalArgumentException("x must be strictly increasing.");
            }
        }
    }

    /**
     * Interpolate a value for the query xq.
     * Clamps to endpoints when xq is outside of the provided domain.
     */
    public double interpolate(double xq) {
        int n = x.length;
        if (xq <= x[0]) {
            return y[0];
        }
        if (xq >= x[n - 1]) {
            return y[n - 1];
        }

        int i = findInterval(xq);
        double x0 = x[i];
        double x1 = x[i + 1];
        double y0 = y[i];
        double y1 = y[i + 1];
        double t = (xq - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    private int findInterval(double xq) {
        int low = 0;
        int high = x.length - 2;
        while (low <= high) {
            int mid = (low + high) >>> 1;
            if (xq < x[mid]) {
                high = mid - 1;
            } else if (xq >= x[mid + 1]) {
                low = mid + 1;
            } else {
                return mid;
            }
        }
        return Math.max(0, Math.min(x.length - 2, low));
    }
}
