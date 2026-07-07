package org.firstinspires.ftc.teamcode.utility.filter;

import org.firstinspires.ftc.teamcode.utility.Vector2D;

/**
 * 使用Vector2D实现的移动平均角度滤波器
 * 利用向量运算避免角度跳变问题
 */
public class AngleWeightedMeanFilter {
    private final int windowSize;
    private final Vector2D[] unitVectors;
    private int index = 0;
    private int count = 0;
    private Vector2D vectorSum = new Vector2D(0, 0);

    public AngleWeightedMeanFilter(int windowSize) {
        if (windowSize <= 0) {
            throw new IllegalArgumentException("windowSize must be > 0");
        }
        this.windowSize = windowSize;
        this.unitVectors = new Vector2D[windowSize];
        for (int i = 0; i < windowSize; i++) {
            unitVectors[i] = new Vector2D(0, 0);
        }
    }

    public double filter(double angleRad,double weight) {
        Vector2D newVector = Vector2D.fromPolar(angleRad, weight);

        if (count < windowSize) {
            unitVectors[index] = newVector;
            if(!Double.isNaN(unitVectors[index].getRadian())) {
                vectorSum = Vector2D.translate(vectorSum, newVector.getX(), newVector.getY());
            }
            count++;
            index = (index + 1) % windowSize;
        } else {
            Vector2D oldest = unitVectors[index];
            if(!Double.isNaN(oldest.getRadian())) {
                vectorSum = Vector2D.translate(vectorSum, -oldest.getX(), -oldest.getY());
            }
            unitVectors[index] = newVector;
            if(!Double.isNaN(newVector.getRadian())) {
                vectorSum = Vector2D.translate(vectorSum, newVector.getX(), newVector.getY());
            }
            index = (index + 1) % windowSize;
        }
        if(Vector2D.equal(vectorSum,Vector2D.ZERO)) return Double.NaN;
        return vectorSum.getRadian();
    }
    public double getAverageAngle() {
        if(Vector2D.equal(vectorSum,Vector2D.ZERO)) return Double.NaN;
        return vectorSum.getRadian();
    }

    /**
     * 添加新角度（度数）并返回平均角度（度数）
     */
    public double filterDegrees(double angleDeg,double weight) {
        double angleRad = Math.toRadians(angleDeg);
        double resultRad = filter(angleRad,weight);
        return Math.toDegrees(resultRad);
    }

    /**
     * 重置滤波器
     */
    public void reset() {
        for (int i = 0; i < windowSize; i++) {
            unitVectors[i].setX(0);
            unitVectors[i].setY(0);
        }
        vectorSum.setX(0);
        vectorSum.setY(0);
        index = 0;
        count = 0;
    }

    /**
     * 获取当前平均角度的向量长度（表示滤波器一致性）
     * 值越接近1，表示角度分布越集中
     */
    public double getConsistency() {
        if (count == 0) return 0;
        return vectorSum.getDistance() / Math.min(count, windowSize);
    }
}