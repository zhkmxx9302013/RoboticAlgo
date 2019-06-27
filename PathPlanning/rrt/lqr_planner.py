import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
import math
import random

show_animation = True

MAX_TIME = 100.0  # Maximum simulation time
DT = 0.1  # Time tick


def LQRplanning(sx, sy, gx, gy):
    """
    LQR规划
    :param sx:
    :param sy:
    :param gx:
    :param gy:
    :return:
    """
    rx, ry = [sx], [sy]

    x = np.array([sx - gx, sy - gy]).reshape(2, 1)  # 状态量(目标距离误差)

    # Linear system model
    A, B = get_system_model()   # 获取状态空间模型

    found_path = False

    time = 0.0
    while time <= MAX_TIME:
        time += DT

        u = LQR_control(A, B, x)

        x = A * x + B * u

        rx.append(x[0, 0] + gx)
        ry.append(x[1, 0] + gy)

        d = math.sqrt((gx - rx[-1])**2 + (gy - ry[-1])**2)
        if d <= 0.1:
            #  print("Goal!!")
            found_path = True
            break

        # animation
        if show_animation:  # pragma: no cover
            plt.plot(sx, sy, "or")
            plt.plot(gx, gy, "ob")
            plt.plot(rx, ry, "-r")
            plt.axis("equal")
            plt.pause(1.0)

    if not found_path:
        print("Cannot found path")
        return [], []

    return rx, ry


def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T * X * A - A.T * X * B * \
            la.inv(R + B.T * X * B) * B.T * X * A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eigVals, eigVecs = la.eig(A - B @ K)

    return K, X, eigVals


def get_system_model():

    A = np.array([[DT, 1.0],
                  [0.0, DT]])
    B = np.array([0.0, 1.0]).reshape(2, 1)

    return A, B


def LQR_control(A, B, x):
    """
    解LQR的反馈矩阵Kopt,求解控制量u
    :param A:
    :param B:
    :param x:
    :return:
    """
    Kopt, X, ev = dlqr(A, B, np.eye(2), np.eye(1)) # A,B,Q,R

    u = -Kopt * x

    return u


def main():
    print(__file__ + " start!!")

    ntest = 10  # number of goal
    area = 100.0  # sampling area

    for i in range(ntest):
        sx = 6.0
        sy = 6.0
        gx = random.uniform(-area, area)
        gy = random.uniform(-area, area)

        rx, ry = LQRplanning(sx, sy, gx, gy)

        if show_animation:  # pragma: no cover
            plt.plot(sx, sy, "or")
            plt.plot(gx, gy, "ob")
            plt.plot(rx, ry, "-r")
            plt.axis("equal")
            plt.pause(1.0)


if __name__ == '__main__':
    main()