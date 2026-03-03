/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "hybrid_a_star/rs_path.h"

#include <glog/logging.h>

// P 371: TABLE 1
const RSPath::RSPathSegmentType RSPath::RS_path_segment_type[18][5] = {
        {L, R, L, N, N},
        {R, L, R, N, N},
        {L, R, L, R, N},
        {R, L, R, L, N},
        {L, R, S, L, N},
        {R, L, S, R, N},
        {L, S, R, L, N},
        {R, S, L, R, N},
        {L, R, S, R, N},
        {R, L, S, L, N},
        {R, S, R, L, N},
        {L, S, L, R, N},
        {L, S, R, N, N},
        {R, S, L, N, N},
        {L, S, L, N, N},//14
        {R, S, R, N, N},
        {L, R, S, L, R},
        {R, L, S, R, L}//17
};

RSPath::RSPath(double turning_radius) : turning_radius_(turning_radius) {}

double RSPath::Mod2Pi(double x) {//把任意角度x归一化到[−π,π]的区间里
    double v = fmod(x, 2 * M_PI);//除以2pai取余
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }//保持到[−π,π]
    return v;
}

void RSPath::Polar(double x, double y, double &r, double &theta) {
    r = std::sqrt(x * x + y * y);//这个就是u
    theta = std::atan2(y, x);//这个就是t
}//u是第一个弯的角度（弧长，因为r归一化为了一），t是直线段的长度

void RSPath::TauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega) {
    double delta = Mod2Pi(u - v);
    double A = std::sin(u) - std::sin(delta);
    double B = std::cos(u) - std::cos(delta) - 1.0;
    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    tau = (t2 < 0.0) ? Mod2Pi(t1 + M_PI) : Mod2Pi(t1);
    omega = Mod2Pi(tau - u + v - phi);
}

bool RSPath::LpSpLp(double x, double y, double phi, double &t, double &u, double &v) {//左转+ 直线+ 左转+
    //x,y是归一化坐标，phi目标点相对起点的朝向差
    Polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);//求u，t

    if (t >= 0.0) {
        v = Mod2Pi(phi - t);//v=phi-t
        if (v >= 0.0) {
            return true;
        }
    }

    return false;
}

bool RSPath::LpSpRp(double x, double y, double phi, double &t, double &u, double &v) {
    double t1, u1;
    Polar(x + std::sin(phi), y - 1 - std::cos(phi), u1, t1);
    u1 = std::pow(u1, 2);

    if (u1 < 4.0) {
        return false;
    }

    double theta;
    u = std::sqrt(u1 - 4.0);
    theta = std::atan2(2.0, u);
    t = Mod2Pi(t1 + theta);
    v = Mod2Pi(t - phi);

    return true;
}

RSPath::RSPathData RSPath::GetRSPath(const double x_0, const double y_0, const double yaw_0,
                                     const double x_1, const double y_1, const double yaw_1) {
    // translation
    double dx = x_1 - x_0;
    double dy = y_1 - y_0;//计算终点相对于起点的向量

    // rotate
    double c = std::cos(yaw_0);// 2d rotation matrix
    double s = std::sin(yaw_0);
    double x = c * dx + s * dy;// x' = cos(yaw0)*dx + sin(yaw0)*dy
    double y = -s * dx + c * dy;// y' = -sin(yaw0)*dx + cos(yaw0)*dy
    //计算旋转矩阵
    //转化为起点为原点的坐标系
    double phi = yaw_1 - yaw_0;//终点朝向在局部坐标系下的角度差，局部坐标系下，起点朝向被视为 0，终点朝向为 phi

    return GetRSPath(x / turning_radius_, y / turning_radius_, phi);//归一化单位半径坐标系（除以转弯半径），phi是角度（朝向差），它本身就是无量纲的，不依赖于半径。转弯半径缩放不会改变角度，所以不用归一化
}

RSPath::RSPathData RSPath::GetRSPath(const double x, const double y, const double phi) {
    RSPathData path;
    CSC(x, y, phi, path);//八种
    CCC(x, y, phi, path);//4种
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);
    //Reeds-Shepp 经典路径类型枚举
    return path;
}

double RSPath::Distance(const double x_0, const double y_0, const double yaw_0,
                        const double x_1, const double y_1, const double yaw_1) {
    return turning_radius_ * GetRSPath(x_0, y_0, yaw_0, x_1, y_1, yaw_1).Length();//计算两点之间 Reeds-Shepp（RS）曲线最短路径长度
}

void RSPath::CSC(double x, double y, double phi, RSPathData &path) {
    //(x, y) → 目标点在局部、单位化坐标系下的位置
    //phi → 目标点相对起点的朝向差
    double t, u, v, length_min = path.Length(), L;//t,u,v 表示一条候选 Reeds–Shepp 路径 的三个段长度（或角度，因为半径归一化后是1，弧长等于半径*角度=角度的数值）。

    if (LpSpLp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//调用 LpSpLp 公式函数，尝试计算一条 L → S → L 类型路径，从起点到终点 /计算长度

        path = RSPathData(RS_path_segment_type[14]/*{L, S, L, N, N}*/, t, u, v);
        length_min = L;
    }

    if (LpSpLp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//利用lsl路径，路径可以相互映射，过 (-x, y, -phi) 的对称，把问题变成了“镜像版”的 LSL；然后把结果参数取负号，恢复到原坐标系，就得到了 RSR 这类路径
        path = RSPathData(RS_path_segment_type[14], -t, -u, -v);
        length_min = L;
    }

    if (LpSpLp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[15], t, u, v);
        length_min = L;
    }

    if (LpSpLp(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[15], -t, -u, -v);
        length_min = L;
    }

    if (LpSpRp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[12], t, u, v);
        length_min = L;
    }

    if (LpSpRp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[12], -t, -u, -v);
        length_min = L;
    }

    if (LpSpRp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[13], t, u, v);
        length_min = L;
    }

    if (LpSpRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[13], -t, -u, -v);
    }
}

bool RSPath::LpRmL(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x - std::sin(phi);
    double eta = y - 1.0 + std::cos(phi);
    double u1, theta;
    Polar(xi, eta, u1, theta);

    if (u1 > 4.0) {
        return false;
    }

    u = -2.0 * std::asin(0.25 * u1);
    t = Mod2Pi(theta + 0.5 * u + M_PI);
    v = Mod2Pi(phi - t + u);

    return true;
}

void RSPath::CCC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length();

    if (LpRmL(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//L+ R- L+
        path = RSPathData(RS_path_segment_type[0], t, u, v);
        length_min = L;
    }

    if (LpRmL(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//L- R+ L-
        path = RSPathData(RS_path_segment_type[0], -t, -u, -v);
        length_min = L;
    }

    if (LpRmL(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//R+ L- R+
        path = RSPathData(RS_path_segment_type[1], t, u, v);
        length_min = L;
    }

    if (LpRmL(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//R- L+ R-
        path = RSPathData(RS_path_segment_type[1], -t, -u, -v);
        length_min = L;
    }

    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);//原始的旋转对称版本
    if (LpRmL(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//L+ R- L+从终点往回走，等价为L- R+ L-
        path = RSPathData(RS_path_segment_type[0], v, u, t);
        length_min = L;
    }

    if (LpRmL(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//L- R+ L-反向行驶加倒xu
        path = RSPathData(RS_path_segment_type[0], -v, -u, -t);
        length_min = L;
    }

    if (LpRmL(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {//R+ L- R+反转
        path = RSPathData(RS_path_segment_type[1], v, u, t);
        length_min = L;
    }

    if (LpRmL(-xb, -yb, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {//R- L+ R-
        path = RSPathData(RS_path_segment_type[1], -v, -u, -t);
    }
}

bool RSPath::LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));

    if (rho > 1.0) {
        return false;
    }

    u = std::acos(rho);
    TauOmega(u, -u, xi, eta, phi, t, v);

    return true;
}

bool RSPath::LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = (20.0 - xi * xi - eta * eta) / 16.0;

    if (rho >= 0.0 && rho <= 1.0) {
        u = -std::acos(rho);
        if (u >= -M_PI_2) {
            TauOmega(u, u, xi, eta, phi, t, v);
            return true;
        }
    }

    return false;
}

void RSPath::CCCC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length();

    if (LpRupLumRm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[2], t, u, -u, v);
        length_min = L;
    }

    if (LpRupLumRm(-x, y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))
            ) {
        path = RSPathData(RS_path_segment_type[2], -t, -u, u, -v);
        length_min = L;
    }

    if (LpRupLumRm(x, -y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))
            ) {
        path = RSPathData(RS_path_segment_type[3], t, u, -u, v);
        length_min = L;
    }

    if (LpRupLumRm(-x, -y, phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))
            ) {
        path = RSPathData(RS_path_segment_type[3], -t, -u, u, -v);
        length_min = L;
    }

    if (LpRumLumRp(x, y, phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))
            ) {
        path = RSPathData(RS_path_segment_type[2], t, u, u, v);
        length_min = L;
    }

    if (LpRumLumRp(-x, y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))
            ) {
        path = RSPathData(RS_path_segment_type[2], -t, -u, -u, -v);
        length_min = L;
    }

    if (LpRumLumRp(x, -y, -phi, t, u, v) &&
        length_min > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))
            ) {
        path = RSPathData(RS_path_segment_type[3], t, u, u, v);
        length_min = L;
    }

    if (LpRumLumRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[3], -t, -u, -u, -v);
    }
}

bool RSPath::LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x - std::sin(phi);
    double eta = y - 1.0 + std::cos(phi);
    double rho, theta;

    Polar(xi, eta, rho, theta);

    if (rho < 2.0) {
        return false;
    }

    double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = Mod2Pi(theta + std::atan2(r, -2.0));
    v = Mod2Pi(phi - M_PI_2 - t);

    return true;
}

bool RSPath::LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho, theta;

    Polar(-eta, xi, rho, theta);

    if (rho < 2.0) {
        return false;
    }

    t = theta;
    u = 2.0 - rho;
    v = Mod2Pi(t + M_PI_2 - phi);

    return true;
}

void RSPath::CCSC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length() - M_PI_2;

    if (LpRmSmLm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[4], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmLm(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[4], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    if (LpRmSmLm(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[5], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmLm(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[5], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    if (LpRmSmRm(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[8], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmRm(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[8], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    if (LpRmSmRm(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[9], t, -M_PI_2, u, v);
        length_min = L;
    }

    if (LpRmSmRm(-x, -y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[9], -t, M_PI_2, -u, -v);
        length_min = L;
    }

    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);
    if (LpRmSmLm(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[6], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmLm(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[6], -v, -u, M_PI_2, -t);
        length_min = L;
    }

    if (LpRmSmLm(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[7], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmLm(-xb, -yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[7], -v, -u, M_PI_2, -t);
        length_min = L;
    }

    if (LpRmSmRm(xb, yb, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[10], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmRm(-xb, yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[10], -v, -u, M_PI_2, -t);
        length_min = L;
    }

    if (LpRmSmRm(xb, -yb, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[11], v, u, -M_PI_2, t);
        length_min = L;
    }

    if (LpRmSmRm(-xb, -yb, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[11], -v, -u, M_PI_2, -t);
    }
}

bool RSPath::LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v) {
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho, theta;

    Polar(xi, eta, rho, theta);

    if (rho >= 2.0) {
        u = 4.0 - std::sqrt(rho * rho - 4.0);

        if (u <= 0.0) {
            t = Mod2Pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = Mod2Pi(t - phi);

            return true;
        }
    }

    return false;
}

void RSPath::CCSCC(double x, double y, double phi, RSPathData &path) {
    double t, u, v, L;
    double length_min = path.Length() - M_PI;
    if (LpRmSLmRp(x, y, phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[16], t, -M_PI_2, u, -M_PI_2, v);
        length_min = L;
    }

    if (LpRmSLmRp(-x, y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[16], -t, M_PI_2, -u, M_PI_2, -v);
        length_min = L;
    }

    if (LpRmSLmRp(x, -y, -phi, t, u, v) && length_min > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
        path = RSPathData(RS_path_segment_type[17], t, -M_PI_2, u, -M_PI_2, v);
        length_min = L;
    }

    if (LpRmSLmRp(-x, -y, phi, t, u, v) && length_min > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
        path = RSPathData(RS_path_segment_type[17], -t, M_PI_2, -u, M_PI_2, -v);
    }
}

TypeVectorVecd<3> RSPath::GetRSPath(const Vec3d &start_state, const Vec3d &goal_state,
                                    const double step_size, double &length) {
    RSPathData rs_path = GetRSPath(start_state.x(), start_state.y(), start_state.z(),
                                   goal_state.x(), goal_state.y(), goal_state.z());//获取从起点到终点的最优 RS 曲线数据

    length = rs_path.Length() * turning_radius_;//计算路径总长度
    // Debug info
//    std::cout << "rs length: " << rs_path.Length() << "  | "
//              << rs_path.length_[0] << " " << rs_path.length_[1] << " "
//              << rs_path.length_[2] << " " << rs_path.length_[3] << " "
//              << rs_path.length_[4] << std::endl;
//    std::cout << "type: "
//              << rs_path.type_[0] << " " << rs_path.type_[1] << " "
//              << rs_path.type_[2] << " " << rs_path.type_[3] << " "
//              << rs_path.type_[4] << std::endl;

    const double path_length = rs_path.Length() * turning_radius_;
    const auto interpolation_number = static_cast<unsigned int> (path_length / step_size);//将整个曲线分为多少步

    double phi;

    TypeVectorVecd<3> path_poses;

    for (unsigned int i = 0; i <= interpolation_number; ++i) {
        double v;
        double t = i * 1.0 / interpolation_number;
        double seg = t * rs_path.Length();//当前步在曲线上的长度

        Vec3d temp_pose(0.0, 0.0, start_state.z());
        for (unsigned int j = 0; j < 5u && seg > 0; ++j) {
            //seg 表示当前插值点对应的曲线长度
            if (rs_path.length_[j]/*当前段长度，就是L，R什么的*/ < 0.0) {//倒退档
                v = std::max(-seg, rs_path.length_[j]);//确定当前插值步在这一段的实际走动量 v
                seg += v;//更新剩余距离 seg
            } else {
                v = std::min(seg, rs_path.length_[j]);
                seg -= v;
            }
            //一段一段走
            phi = temp_pose.z();//根据路径段的类型取更新角度
            switch (rs_path.type_[j]) {
                case L:
                    temp_pose.x() = std::sin(phi + v) - std::sin(phi) + temp_pose.x();
                    temp_pose.y() = -std::cos(phi + v) + std::cos(phi) + temp_pose.y();//端点坐标
                    temp_pose.z() = phi + v;
                    break;
                case R:
                    temp_pose.x() = -std::sin(phi - v) + std::sin(phi) + temp_pose.x();
                    temp_pose.y() = std::cos(phi - v) - std::cos(phi) + temp_pose.y();
                    temp_pose.z() = phi - v;
                    break;
                case S:
                    temp_pose.x() = v * std::cos(phi) + temp_pose.x();
                    temp_pose.y() = v * std::sin(phi) + temp_pose.y();
                    temp_pose.z() = phi;
                    break;
                case N:
                    break;
            }
        }
        //从起点累积生成曲线上每个离散点的局部位姿

        Vec3d pose;
        pose.block<2, 1>(0, 0) = temp_pose.block<2, 1>(0, 0) * turning_radius_//将前面累积的局部x，y（单位半径下）缩放回实际 
                                 + start_state.block<2, 1>(0, 0);//加上起点坐标
        pose.z() = temp_pose.z();

        path_poses.emplace_back(pose);//将转换后的全局点加入路径列表 path_poses,最终 path_poses 就是 离散的全局 RS 曲线轨迹点
    }

    return path_poses;//离散的全局 RS 曲线轨迹点
}