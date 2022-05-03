/*
 * Shuster, S.D. Oh - Three-axis Attitude Determination from Vector Observations (1981)
 * Rishav (2020/12/7)
 */

#ifndef _QUEST_H_
#define _QUEST_H_

#include "lanczos.h"

class quest
{
private:
    uint8_t N;

public:
    quest(const uint8_t n);
    Quaternion update(const Vector<3> &i1, const Vector<3> &i2, const Vector<3> &b1, const Vector<3> &b2);
    Matrix<3, 3> get_dcm(const Vector<3> &i1, const Vector<3> &i2, const Vector<3> &b1, const Vector<3> &b2);
};

quest::quest(const uint8_t n)
{
    N = n;
}

Quaternion quest::update(const Vector<3> &i1, const Vector<3> &i2, const Vector<3> &b1, const Vector<3> &b2)
{
    double w[2] = {1 / var(b1), 1 / var(b2)};
    double tolerance = 10e-5;

    Vector<3> v_i[N];
    Vector<3> v_b[N];
    v_i[0] = i1;
    v_i[1] = i2;
    v_b[0] = b1;
    v_b[1] = b2;

    Matrix<3, 3> B;
    for (uint8_t i_iters = 0; i_iters < 2; i_iters++)
    {
        B = B + w[i_iters] * vvT(v_b[i_iters], v_i[i_iters]);
    }

    Matrix<3, 3> S = B + trans(B);
    double delta = det(S);
    double sigma = trace(B);
    double kappa = trace(delta * inv(S));
    Vector<3> Z({B(1, 2) - B(2, 1), B(2, 0) - B(0, 2), B(0, 1) - B(1, 0)});

    double a = sigma * sigma - kappa;
    double b = sigma * sigma + dot(Z, Z);
    double c = delta + dot(Z, S * Z);
    double d = dot(Z, S * S * Z);
    double constant = a * b + c * sigma - d;

    double lambda = w[0] + w[1];
    double last_lambda = 0.0;
    while (fabs(lambda - last_lambda) / lambda >= tolerance)
    {
        last_lambda = lambda;
        lambda = lambda - (((lambda * lambda - (a + b)) * lambda - c) * lambda + constant) /
                              (2.0 * (2.0 * lambda * lambda - (a + b)) * lambda - c);
    }
    
    double alpha = lambda * lambda - sigma * sigma + kappa;
    double beta = lambda - sigma;
    double gamma = (lambda + sigma) * alpha - delta;

    Vector<4> q_opt;
    Matrix<3, 3> I({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    Vector<3> X;

    // Optimal quaternion
    X = (alpha * I + beta * S + S * S) * Z;
    q_opt.set(1, X(0));
    q_opt.set(2, X(1));
    q_opt.set(3, X(2));
    q_opt.set(0, gamma);
    q_opt = q_opt / sqrt(gamma * gamma + norm(X) * norm(X));
    Quaternion q(q_opt);

    return q;
}

Matrix<3, 3> quest::get_dcm(const Vector<3> &i1, const Vector<3> &i2, const Vector<3> &b1, const Vector<3> &b2)
{
    Quaternion q_optimal;
    q_optimal = update(i1, i2, b1, b2);

    return q_optimal.get_dcm();
}

#endif // quest.h