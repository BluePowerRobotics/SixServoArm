# filter 数据滤波算法

## 概述

本目录包含数据滤波相关的工具类，用于处理传感器数据的噪声过滤和状态估计。

## 文件列表

| 文件 | 功能说明 |
|------|----------|
| `MeanFilter.java` | 移动平均滤波器 |
| `AngleMeanFilter.java` | 角度专用移动平均滤波器（避免角度跳变） |
| `AngleWeightedMeanFilter.java` | 加权角度移动平均滤波器 |
| `kalmanfilter/OneDimensionKalmanFilter.java` | 一维卡尔曼滤波器 |
| `kalmanfilter/PosVelTuple.java` | 位置-速度数据结构 |
| `kalmanfilter/jama/` | JAMA矩阵运算库 |

## 滤波器选择指南

| 场景 | 推荐滤波器 | 原因 |
|------|-----------|------|
| 普通传感器数据滤波 | `MeanFilter` | 简单高效，适用于大多数场景 |
| 角度数据滤波 | `AngleMeanFilter` | 处理角度周期性，避免±180°跳变 |
| 带权重的角度滤波 | `AngleWeightedMeanFilter` | 可根据置信度加权 |
| 需要状态估计（位置+速度） | `OneDimensionKalmanFilter` | 最优估计，融合多传感器 |

## MeanFilter 使用说明

### 功能特性

- 滑动窗口移动平均
- 自动处理NaN和无穷大值
- 提供方差计算

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;

// 创建窗口大小为10的滤波器
MeanFilter filter = new MeanFilter(10);

// 逐步添加数据并获取滤波后的值
double filteredValue = filter.filter(rawSensorValue);

// 获取当前均值
double mean = filter.getMean();

// 获取方差
double variance = filter.getVariance();

// 重置滤波器
filter.reset();
```

### 方法签名

| 方法 | 说明 |
|------|------|
| `MeanFilter(windowSize)` | 构造函数 |
| `filter(newValue)` | 添加新样本并返回滤波后值 |
| `getMean()` | 获取当前均值 |
| `getVariance()` | 获取当前方差 |
| `reset()` | 重置滤波器 |

## AngleMeanFilter 使用说明

### 功能特性

- 使用向量平均避免角度跳变问题
- 支持弧度和角度两种输入方式
- 提供一致性指标

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.AngleMeanFilter;

// 创建窗口大小为5的角度滤波器
AngleMeanFilter angleFilter = new AngleMeanFilter(5);

// 输入弧度
double filteredRad = angleFilter.filter(angleRad);

// 输入角度（度数）
double filteredDeg = angleFilter.filterDegrees(angleDeg);

// 获取一致性指标（0-1，越接近1越一致）
double consistency = angleFilter.getConsistency();
```

### 方法签名

| 方法 | 说明 |
|------|------|
| `AngleMeanFilter(windowSize)` | 构造函数 |
| `filter(angleRad)` | 添加弧度值并返回平均角度 |
| `filterDegrees(angleDeg)` | 添加角度值并返回平均角度 |
| `getAverageAngle()` | 获取当前平均角度 |
| `getConsistency()` | 获取一致性指标 |
| `reset()` | 重置滤波器 |

## AngleWeightedMeanFilter 使用说明

### 功能特性

- 支持加权角度平均
- 可根据传感器置信度动态调整权重

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.AngleWeightedMeanFilter;

AngleWeightedMeanFilter filter = new AngleWeightedMeanFilter(5);

// 带权重的滤波（权重范围建议0-1）
double filtered = filter.filter(angleRad, confidence);
```

## OneDimensionKalmanFilter 使用说明

### 功能特性

- 一维卡尔曼滤波，融合轮式编码器和视觉定位
- 支持Dashboard实时调参
- 状态包含位置和速度

### 使用示例

```java
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.OneDimensionKalmanFilter;
import org.firstinspires.ftc.teamcode.utility.filter.kalmanfilter.PosVelTuple;

// 创建卡尔曼滤波器（初始位置和速度）
OneDimensionKalmanFilter kf = new OneDimensionKalmanFilter(0, 0);

// 更新滤波器
// 参数：轮式编码器位置，视觉测量位置（无测量时传入Double.NaN）
PosVelTuple result = kf.Update(wheelPosition, visionPosition);

double estimatedPosition = result.position;
double estimatedVelocity = result.velocity;

// 重置滤波器
kf.reset(newPosition, newVelocity);
```

### 可调参数（Dashboard）

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `q_pos` | 位置过程噪声 | 1e-4 |
| `q_vel` | 速度过程噪声 | 1e-6 |
| `R` | 测量噪声方差 | 0.0001 |

### 注意事项

1. 卡尔曼滤波器使用匀速运动模型
2. 时间间隔自动计算，无需手动设置
3. 视觉测量可选，无测量时传入 `Double.NaN`

## JAMA 矩阵库

`jama/` 目录包含 JAMA 矩阵运算库，用于卡尔曼滤波中的矩阵计算：

| 文件 | 功能说明 |
|------|----------|
| `Matrix.java` | 矩阵类 |
| `LUDecomposition.java` | LU分解 |
| `QRDecomposition.java` | QR分解 |
| `CholeskyDecomposition.java` | Cholesky分解 |
| `EigenvalueDecomposition.java` | 特征值分解 |
| `SingularValueDecomposition.java` | SVD分解 |

这些类通常不需要直接使用，由卡尔曼滤波器内部调用。