#pragma once

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

/**
 * @brief 硬阈值损失函数
 * 当残差小于阈值时，权重为1；超过阈值时，权重为0
 * 用于水下SLAM中剔除虚无区域的误匹配特征点
 * 
 * rho(s) = s,           if s < threshold^2
 *        = threshold^2, if s >= threshold^2
 * 
 * 导数：
 * rho'(s) = 1,  if s < threshold^2
 *         = 0,  if s >= threshold^2
 */
class HardThresholdLoss : public ceres::LossFunction
{
public:
    explicit HardThresholdLoss(double threshold) : threshold_sq_(threshold * threshold) {}

    virtual void Evaluate(double s, double rho[3]) const
    {
        if (s < threshold_sq_)
        {
            // 残差小于阈值：正常计算
            rho[0] = s;              // rho(s)
            rho[1] = 1.0;            // rho'(s)
            rho[2] = 0.0;            // rho''(s)
        }
        else
        {
            // 残差超过阈值：设为常数，梯度为0（不参与优化）
            rho[0] = threshold_sq_;  // rho(s) = constant
            rho[1] = 0.0;            // rho'(s) = 0 (zero gradient)
            rho[2] = 0.0;            // rho''(s) = 0
        }
    }

private:
    const double threshold_sq_;  // 阈值的平方
};


/**
 * @brief 饱和损失函数（Saturated Loss）
 * 比硬阈值更平滑的过渡，在阈值附近逐渐降低权重
 * 
 * rho(s) = 2*threshold^2 * (sqrt(1 + s/threshold^2) - 1)
 * 
 * 当 s -> 0时，rho(s) ≈ s（接近二次损失）
 * 当 s -> ∞时，rho(s) ≈ 2*threshold^2 * sqrt(s/threshold^2) = 2*threshold*sqrt(s)（线性增长）
 */
class SaturatedLoss : public ceres::LossFunction
{
public:
    explicit SaturatedLoss(double threshold) : a_(threshold * threshold) {}

    virtual void Evaluate(double s, double rho[3]) const
    {
        const double sum = 1.0 + s / a_;
        const double tmp = std::sqrt(sum);
        
        // rho(s) = 2*a*(sqrt(1 + s/a) - 1)
        rho[0] = 2.0 * a_ * (tmp - 1.0);
        
        // rho'(s) = 1 / sqrt(1 + s/a)
        rho[1] = 1.0 / tmp;
        
        // rho''(s) = -1 / (2*a*(1 + s/a)^(3/2))
        rho[2] = -0.5 / (a_ * sum * tmp);
    }

private:
    const double a_;  // threshold^2
};
