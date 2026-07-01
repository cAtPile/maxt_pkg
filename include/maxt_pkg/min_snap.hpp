#ifndef MAXT_MIN_SNAP_HPP
#define MAXT_MIN_SNAP_HPP

#include <cmath>

namespace maxt {

class MinSnapCurve {
public:
    struct Boundary {
        double pos{0}, vel{0}, acc{0};
    };

    MinSnapCurve() = default;

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
        Boundary s{x0, 0, 0}, e{x1, 0, 0};
        setBoundaries(s, e, T, {y0, 0, 0}, {y1, 0, 0}, {z0, 0, 0}, {z1, 0, 0});
    }

    void evaluate(double t,
                  double& px, double& py, double& pz,
                  double& vx, double& vy, double& vz,
                  double& ax, double& ay, double& az) const
    {
        px = evalPos(cx_, t); py = evalPos(cy_, t); pz = evalPos(cz_, t);
        vx = evalVel(cx_, t); vy = evalVel(cy_, t); vz = evalVel(cz_, t);
        ax = evalAcc(cx_, t); ay = evalAcc(cy_, t); az = evalAcc(cz_, t);
    }

    /// p(t) = c0 + c1·t + c2·t² + c3·t³ + c4·t⁴ + c5·t⁵ + c6·t⁶ + c7·t⁷
    /// Minimizes ∫(p'''')² dt subject to p,v,a at t=0 and t=T.
    /// Result: c4 = 0 (natural consequence of min-snap).
    static void solveAxis(double p0, double v0, double a0,
                          double pf, double vf, double af,
                          double T, double c[8])
    {
        double T2 = T * T, T3 = T2 * T, T5, T6, T7;

        c[0] = p0;
        c[1] = v0;
        c[2] = a0 / 2.0;

        double dp = pf - p0 - v0 * T - c[2] * T2;
        double dv = vf - v0 - a0 * T;
        double da = af - a0;

        double dvT  = dv * T;
        double daT2 = da * T2;

        T3 = T2 * T;
        T5 = T3 * T2;
        T6 = T5 * T;
        T7 = T6 * T;

        c[3] = ( 21.0 * dp -  8.0 * dvT +       daT2)  / (3.0 * T3);
        c[4] = 0.0;
        c[5] = (-21.0 * dp + 10.0 * dvT - 1.5 * daT2)  / T5;
        c[6] = ( 63.0 * dp - 31.0 * dvT +  5.0 * daT2)  / (3.0 * T6);
        c[7] = ( -6.0 * dp +  3.0 * dvT - 0.5 * daT2)  / T7;
    }

    static double evalPos(const double c[8], double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        double t5 = t4 * t, t6 = t5 * t, t7 = t6 * t;
        return c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4
             + c[5]*t5 + c[6]*t6 + c[7]*t7;
    }

    static double evalVel(const double c[8], double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        double t5 = t4 * t, t6 = t5 * t;
        return c[1] + 2*c[2]*t + 3*c[3]*t2 + 4*c[4]*t3
             + 5*c[5]*t4 + 6*c[6]*t5 + 7*c[7]*t6;
    }

    static double evalAcc(const double c[8], double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return 2*c[2] + 6*c[3]*t + 12*c[4]*t2
             + 20*c[5]*t3 + 30*c[6]*t4 + 42*c[7]*t5;
    }

    static double evalJerk(const double c[8], double t) {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        return 6*c[3] + 24*c[4]*t + 60*c[5]*t2
             + 120*c[6]*t3 + 210*c[7]*t4;
    }

    static double evalSnap(const double c[8], double t) {
        double t2 = t * t, t3 = t2 * t;
        return 24*c[4] + 120*c[5]*t + 360*c[6]*t2 + 840*c[7]*t3;
    }

    double duration() const { return T_; }

private:
    double cx_[8]{}, cy_[8]{}, cz_[8]{};
    double T_{1.0};
};

} // namespace maxt
#endif
