package org.firstinspires.ftc.teamcode.utility.PID;

/**
 * PIDController类实现了基本的PID控制器功能
 * 用于根据目标值和测量值计算控制输出
 */
public class PIDController {
    /** 比例系数 */
    private double kP;
    /** 积分系数 */
    private double kI;
    /** 微分系数 */
    private double kD;
    /** 积分值 */
    private double integral;
    /** 上一次的误差值 */
    private double previousError;
    /** 积分上限 */
    private double maxI = 1;
    /** Izone 区域，误差小于该值才累加I */
    private double iZone = Double.POSITIVE_INFINITY;

    /**
     * 构造函数，初始化PID参数
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     */
    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 1, Double.POSITIVE_INFINITY);
    }

    /**
     * 构造函数，初始化PID参数和积分上限
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     * @param maxI 积分上限
     * @param iZone 误差进入该范围后再开始累加i
     */
    public PIDController(double kP, double kI, double kD, double maxI, double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
        this.iZone = iZone;
        this.integral = 0;
        this.previousError = 0;
    }

    /**
     * 计算PID控制器输出
     * @param setpoint 目标值
     * @param measurement 当前测量值
     * @param dt 时间间隔（秒）
     * @return PID控制器输出
     */
    public double calculate(double setpoint, double measurement, double dt) {
        // 计算误差
        double error = setpoint - measurement;
        // 仅在误差小于Izone时才累加积分
        if (Math.abs(error) < iZone) {
            integral += error * dt;
            // 积分限幅
            if (integral > maxI) {
                integral = maxI;
            } else if (integral < -maxI) {
                integral = -maxI;
            }
        } else {
            integral = 0; // 误差超出Izone时清零积分
        }
        // 计算微分
        double derivative = (error - previousError) / dt;

        // 更新上一次误差
        previousError = error;

        // 计算并返回PID输出
        return kP * error + kI * integral + kD * derivative;
    }

    /**
     * 重置控制器状态
     * 清零积分值和上一次误差
     */
    public void reset() {
        integral = 0;
        previousError = 0;
    }

    /**
     * 设置PID参数
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        // 如果积分系数为0，清零积分值
        if(kI==0) integral=0;
    }

    /**
     * 设置积分上限
     * @param maxI 积分上限
     */
    public void setMaxI(double maxI) {
        this.maxI = maxI;
    }

    /**
     * 设置Izone区域，误差小于该值才累加I
     * @param iZone Izone阈值
     */
    public void setIZone(double iZone) {
        this.iZone = iZone;
    }

    /**
     * 获取当前Izone阈值
     * @return Izone阈值
     */
    public double getIZone() {
        return iZone;
    }
}