#ifndef MAXT_QUINTIC_CURVE_HPP
#define MAXT_QUINTIC_CURVE_HPP

#include <cmath>

namespace maxt {

class QuinticCurve {
public:
    struct Boundary {
        double pos{0.0}, vel{0.0}, acc{0.0};
    };

    QuinticCurve() = default;

    void setBoundaries(const Boundary& sx, const Boundary& ex, double T,
                       const Boundary& sy, const Boundary& ey,
                       const Boundary& sz, const Boundary& ez)
    {
        T_ = T;
        solveAxis(sx.pos, sx.vel, sx.acc, ex.pos, ex.vel, ex.acc, T, cx_);
        solveAxis(sy.pos, sy.vel, sy.acc, ey.pos, ey.vel, ey.acc, T, cy_);
        solveAxis(sz.pos, sz.vel, sz.acc, ez.pos, ez.vel, ez.acc, T, cz_);
    }

    void setWaypoints(double x0, double y0, double z0,
                      double x1, double y1, double z1, double T)
    {
        Boundary sx{x0, 0, 0}, ex{x1, 0, 0};
        Boundary sy{y0, 0, 0}, ey{y1, 0, 0};
        Boundary sz{z0, 0, 0}, ez{z1, 0, 0};
        setBoundaries(sx, ex, T, sy, ey, sz, ez);
    }

    void evaluate(double t,
                  double& px, double& py, double& pz,
                  double& vx, double& vy, double& vz,
                  double& ax, double& ay, double& az) const
    {
        px = evalPos(cx_, t);
        py = evalPos(cy_, t);
        pz = evalPos(cz_, t);
        vx = evalVel(cx_, t);
        vy = evalVel(cy_, t);
        vz = evalVel(cz_, t);
        ax = evalAcc(cx_, t);
        ay = evalAcc(cy_, t);
        az = evalAcc(cz_, t);
    }

    /// Solve quintic coefficients for a single axis.
    /// p(t) = c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 + c[4]*t^4 + c[5]*t^5
    /// Boundary conditions: p(0)=p0, p'(0)=v0, p''(0)=a0,
    ///                      p(T)=pf, p'(T)=vf, p''(T)=af
    static void solveAxis(double p0, double v0, double a0,
                          double pf, double vf, double af,
                          double T, double c[6])
    {
        c[0] = p0;
        c[1] = v0;
        c[2] = a0 / 2.0;

        double dp = pf - p0 - v0 * T - c[2] * T * T;
        double dv = vf - v0 - a0 * T;
        double da = af - a0;

        double dvT = T * dv;
        double daT2 = T * T * da;

        double b5 = (daT2 + 12.0 * dp - 6.0 * dvT) / 2.0;
        double b4 = dvT - 3.0 * dp - 2.0 * b5;
        double b3 = dp - b4 - b5;

        double T3 = T * T * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        c[3] = b3 / T3;
        c[4] = b4 / T4;
        c[5] = b5 / T5;
    }

    static double evalPos(const double c[6], double t)
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        return c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4 + c[5]*t5;
    }

    static double evalVel(const double c[6], double t)
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        return c[1] + 2.0*c[2]*t + 3.0*c[3]*t2 + 4.0*c[4]*t3 + 5.0*c[5]*t4;
    }

    static double evalAcc(const double c[6], double t)
    {
        double t2 = t * t;
        double t3 = t2 * t;
        return 2.0*c[2] + 6.0*c[3]*t + 12.0*c[4]*t2 + 20.0*c[5]*t3;
    }

    static double evalJerk(const double c[6], double t)
    {
        double t2 = t * t;
        return 6.0*c[3] + 24.0*c[4]*t + 60.0*c[5]*t2;
    }

    double duration() const { return T_; }

private:
    double cx_[6]{}, cy_[6]{}, cz_[6]{};
    double T_{1.0};
};

} // namespace maxt

#endif
