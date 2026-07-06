package org.firstinspires.ftc.teamcode.utility.filter.ESKF;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;

import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.jama.Matrix;

/**
 * 3-DoF 误差状态卡尔曼滤波器（Error-State Kalman Filter）
 *
 * <p><b>原理说明</b></p>
 * ESKF 将真实状态分解为“名义状态”和“误差状态”：
 * <pre>
 *   true_state = nominal_state ⊕ error_state
 * </pre>
 * 对于平面位姿（x, y, θ），⊕ 定义为普通加法（位置直接相加，航向角相加后归一化到 [-π,π)）。
 * 误差状态是一个小量，假设服从零均值高斯分布，其协方差矩阵 P 描述不确定性。
 *
 * <p><b>预测步骤</b></p>
 * 名义状态通过运动模型（如轮式里程计增量）更新。误差状态的协方差按线性化模型传播：
 * <pre>
 *   P_new = F * P_old * F^T + Q
 * </pre>
 * 当运动模型为简单的“名义状态 += 位姿增量”时，F = I（单位矩阵），Q 为过程噪声协方差。
 *
 * <p><b>更新步骤</b></p>
 * 利用绝对观测（如视觉位姿）对误差状态进行卡尔曼更新。观测模型假设为：
 * <pre>
 *   z = nominal_state ⊕ error_state + v,  v ~ N(0, R)
 * </pre>
 * 由于观测直接是位姿，观测矩阵 H = I。新息 y = z ⊖ nominal_state，计算卡尔曼增益 K，
 * 修正误差状态，再注入名义状态。支持多次迭代以改善线性化误差。
 *
 * <p><b>使用方式</b></p>
 * <ol>
 *   <li>创建 ESKF 实例，给定初始位姿和初始协方差（通常对角阵表示初始不确定性）。</li>
 *   <li>每帧根据里程计增量调用 {@link #predict(Pose2d, Matrix)}，同时传入过程噪声 Q（可自适应调整）。</li>
 *   <li>当获得有效视觉位姿时，调用 {@link #update(Pose2d, Matrix, int)} 进行修正。</li>
 *   <li>通过 {@link #getNominalState()} 获取当前最优估计。</li>
 * </ol>
 *
 * @see <a href="https://arxiv.org/abs/1711.02508">Quaternion kinematics for the error-state Kalman filter (Solà, 2017)</a>
 */
public class ESKF {
    /** 名义状态 [x, y, θ] */
    private Pose2d nominalState;

    /** 误差状态协方差矩阵 (3x3) */
    private Matrix covariance;

    /** 单位矩阵 (3x3)，用于简化运算 */
    private static final Matrix IDENTITY_3 = Matrix.identity(3, 3);

    /**
     * 构造函数
     *
     * @param initialPose        初始名义位姿
     * @param initialCovariance  初始协方差矩阵（通常对角阵，如 diag(0.01, 0.01, 0.001)）
     */
    public ESKF(Pose2d initialPose, Matrix initialCovariance) {
        this.nominalState = initialPose;
        this.covariance = initialCovariance.copy(); // 防止外部修改
    }

    /**
     * 预测步骤（里程计驱动）
     *
     * @param deltaPose         世界坐标系下的位姿增量（Δx, Δy, Δθ）
     * @param processNoiseCov   过程噪声协方差矩阵 Q (3x3)
     */
    public void predict(Pose2d deltaPose, Matrix processNoiseCov) {
        // 1. 更新名义状态：直接加法
        nominalState = posePlusDelta(nominalState, deltaPose);

        // 2. 误差状态协方差预测：P = F * P * F^T + Q
        //    对于简单的加法模型，F = I，因此 P = P + Q
        covariance = covariance.plus(processNoiseCov);
    }

    /**
     * 更新步骤（视觉观测）
     * 使用迭代误差状态卡尔曼更新，固定迭代次数。
     *
     * @param observation        观测位姿 (x, y, θ)
     * @param observationNoiseCov 观测噪声协方差矩阵 R (3x3)
     * @param maxIterations      最大迭代次数（通常设为 1，特殊情况下 2）
     */
    public void update(Pose2d observation, Matrix observationNoiseCov, int maxIterations) {
        Pose2d currentNominal = nominalState; // 迭代中会更新

        for (int iter = 0; iter < maxIterations; iter++) {
            // 1. 计算新息 (innovation) y = z ⊖ currentNominal
            Matrix y = computeInnovation(observation, currentNominal);

            // 2. 观测矩阵 H = I (3x3)
            Matrix H = IDENTITY_3;

            // 3. 新息协方差 S = H * P * H^T + R
            Matrix S = H.times(covariance).times(H.transpose()).plus(observationNoiseCov);

            // 4. 卡尔曼增益 K = P * H^T * inv(S)
            Matrix K = covariance.times(H.transpose()).times(S.inverse());

            // 5. 误差状态修正量 dx = K * y
            Matrix dx = K.times(y);

            // 6. 注入名义状态：currentNominal = currentNominal ⊕ dx
            currentNominal = injectCorrection(currentNominal, dx);

            // 7. 更新协方差：P = (I - K*H) * P
            Matrix I_KH = IDENTITY_3.minus(K.times(H));
            covariance = I_KH.times(covariance);
        }

        // 将最终的名义状态赋给成员变量
        nominalState = currentNominal;
    }

    /**
     * 获取当前名义状态（最优估计）
     */
    public Pose2d getNominalState() {
        return nominalState;
    }

    /**
     * 设置名义状态（通常用于重置）
     */
    public void setNominalState(Pose2d pose) {
        this.nominalState = pose;
    }

    /**
     * 获取误差状态协方差矩阵
     */
    public Matrix getCovariance() {
        return covariance.copy();
    }

    /**
     * 重置滤波器
     *
     * @param initialPose        初始位姿
     * @param initialCovariance  初始协方差
     */
    public void reset(Pose2d initialPose, Matrix initialCovariance) {
        this.nominalState = initialPose;
        this.covariance = initialCovariance.copy();
    }

    // ==================== 私有辅助方法 ====================

    /**
     * 计算新息：y = z ⊖ nominal
     * 位置直接相减，角度差归一化到 [-π, π)。
     *
     * @param z       观测位姿
     * @param nominal 名义位姿
     * @return 3x1 矩阵 [Δx, Δy, Δθ]^T
     */
    private Matrix computeInnovation(Pose2d z, Pose2d nominal) {
        double dx = z.position.x - nominal.position.x;
        double dy = z.position.y - nominal.position.y;
        double dTheta = normalizeAngle(z.heading.toDouble() - nominal.heading.toDouble());
        return new Matrix(new double[][]{{dx}, {dy}, {dTheta}});
    }

    /**
     * 位姿加法：pose ⊕ delta，其中 delta 为 [Δx, Δy, Δθ]^T 矩阵。
     *
     * @param pose  原体位姿
     * @param delta 3x1 矩阵
     * @return 新位姿
     */
    private Pose2d posePlusDelta(Pose2d pose, Matrix delta) {
        if (delta.getRowDimension() != 3 || delta.getColumnDimension() != 1) {
            throw new IllegalArgumentException("delta 必须是 3x1 矩阵");
        }
        double dx = delta.get(0, 0);
        double dy = delta.get(1, 0);
        double dTheta = delta.get(2, 0);
        return new Pose2d(
                pose.position.x + dx,
                pose.position.y + dy,
                normalizeAngle(pose.heading.toDouble() + dTheta)
        );
    }

    /**
     * 位姿加法：pose ⊕ deltaPose，其中 deltaPose 为另一个 Pose2d。
     */
    private Pose2d posePlusDelta(Pose2d pose, Pose2d deltaPose) {
        return new Pose2d(
                pose.position.x + deltaPose.position.x,
                pose.position.y + deltaPose.position.y,
                normalizeAngle(pose.heading.toDouble() + deltaPose.heading.toDouble())
        );
    }

    /**
     * 将误差状态修正量注入名义状态：nominal = nominal ⊕ dx
     */
    private Pose2d injectCorrection(Pose2d nominal, Matrix dx) {
        return posePlusDelta(nominal, dx);
    }

    /**
     * 将角度归一化到 [-π, π)
     * 修正负数取模的边界情况，确保稳定性。
     */
    private double normalizeAngle(double angle) {
        // 取模，结果在 [0, 2π) 范围内
        angle %= 2 * Math.PI;
        if (angle < 0) angle += 2 * Math.PI;
        // 映射到 [-π, π)
        if (angle >= Math.PI) angle -= 2 * Math.PI;
        return angle;
    }
}