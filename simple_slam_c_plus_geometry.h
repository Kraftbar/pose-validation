#ifndef SIMPLE_SLAM_C_PLUS_GEOMETRY_H
#define SIMPLE_SLAM_C_PLUS_GEOMETRY_H

// Cyclic Jacobi sweeps with a relative convergence criterion. A fixed budget
// of 100 individual rotations is insufficient for a 9x9 or 12x12 DLT system.
static void geometry_eigen(const double *A, int n, double *W, double *V) {
    double M[144], scale = 0;
    for (int i = 0; i < n*n; i++)
        if (fabs(A[i]) > scale) scale = fabs(A[i]);
    for (int i = 0; i < n*n; i++) {
        M[i] = scale > 0 ? A[i]/scale : 0;
        V[i] = (i/n == i%n);
    }
    for (int sweep = 0; sweep < 50; sweep++) {
        double off = 0;
        for (int p = 0; p < n-1; p++) for (int q = p+1; q < n; q++) {
            double g = M[p*n+q];
            if (fabs(g) > off) off = fabs(g);
            if (fabs(g) < DBL_EPSILON) continue;
            double z = (M[q*n+q]-M[p*n+p])/(2*g);
            double t = copysign(1.0,z)/(fabs(z)+hypot(1.0,z));
            double c = 1/sqrt(1+t*t), s = c*t;
            M[p*n+p] -= t*g;
            M[q*n+q] += t*g;
            M[p*n+q] = M[q*n+p] = 0;
            for (int k = 0; k < n; k++) {
                if (k != p && k != q) {
                    double x = M[k*n+p], y = M[k*n+q];
                    M[k*n+p] = M[p*n+k] = c*x-s*y;
                    M[k*n+q] = M[q*n+k] = s*x+c*y;
                }
                double x = V[k*n+p], y = V[k*n+q];
                V[k*n+p] = c*x-s*y;
                V[k*n+q] = s*x+c*y;
            }
        }
        if (off < 8*DBL_EPSILON) break;
    }
    for (int i = 0; i < n; i++) W[i] = M[i*n+i]*scale;
}

// One-sided Jacobi SVD: A = U diag(s) V^T, s in descending order.
// Work on A itself so a rank-two essential matrix retains its null space.
static void geometry_svd3(const double A[9], double s[3], double U[9], double V[9]) {
    double B[9], scale = 0;
    for (int i = 0; i < 9; i++)
        if (fabs(A[i]) > scale) scale = fabs(A[i]);
    for (int i = 0; i < 9; i++) {
        B[i] = scale > 0 ? A[i] / scale : 0;
        V[i] = (i % 4 == 0);
    }
    for (int sweep = 0; sweep < 32; sweep++) {
        int changed = 0;
        for (int p = 0; p < 2; p++) for (int q = p + 1; q < 3; q++) {
            double a = 0, b = 0, g = 0;
            for (int r = 0; r < 3; r++) {
                a += B[3*r+p] * B[3*r+p];
                b += B[3*r+q] * B[3*r+q];
                g += B[3*r+p] * B[3*r+q];
            }
            if (fabs(g) <= 8*DBL_EPSILON*sqrt(a*b)) continue;
            double z = (b-a)/(2*g);
            double t = copysign(1.0, z)/(fabs(z) + hypot(1.0, z));
            double c = 1/sqrt(1+t*t), sn = c*t;
            for (int r = 0; r < 3; r++) {
                double x = B[3*r+p], y = B[3*r+q];
                B[3*r+p] = c*x - sn*y;
                B[3*r+q] = sn*x + c*y;
                x = V[3*r+p]; y = V[3*r+q];
                V[3*r+p] = c*x - sn*y;
                V[3*r+q] = sn*x + c*y;
            }
            changed = 1;
        }
        if (!changed) break;
    }
    for (int j = 0; j < 3; j++)
        s[j] = hypot(hypot(B[j], B[3+j]), B[6+j]);
    for (int p = 0; p < 2; p++) for (int q = p+1; q < 3; q++)
        if (s[q] > s[p]) {
            double tmp = s[p]; s[p] = s[q]; s[q] = tmp;
            for (int r = 0; r < 3; r++) {
                tmp = B[3*r+p]; B[3*r+p] = B[3*r+q]; B[3*r+q] = tmp;
                tmp = V[3*r+p]; V[3*r+p] = V[3*r+q]; V[3*r+q] = tmp;
            }
        }
    for (int j = 0; j < 3; j++) {
        double v[3] = {B[j], B[3+j], B[6+j]};
        if (s[j] <= 32*DBL_EPSILON*s[0] || s[j] == 0) {
            // Complete an orthonormal basis even for rank-one/zero inputs.
            int axis = 0;
            double best = -1;
            for (int k = 0; k < 3; k++) {
                double rem = 1;
                for (int l = 0; l < j; l++) rem -= U[3*k+l]*U[3*k+l];
                if (rem > best) { best = rem; axis = k; }
            }
            for (int k = 0; k < 3; k++) v[k] = (k == axis);
        }
        for (int l = 0; l < j; l++) {
            double dot = 0;
            for (int k = 0; k < 3; k++) dot += v[k]*U[3*k+l];
            for (int k = 0; k < 3; k++) v[k] -= dot*U[3*k+l];
        }
        double norm = hypot(hypot(v[0], v[1]), v[2]);
        for (int k = 0; k < 3; k++) U[3*k+j] = v[k]/norm;
    }
    for (int j = 0; j < 3; j++) s[j] *= scale;
}

// Left perturbation: R' = exp([w]x) R, t' = exp([w]x) t + v.
// This is the retraction differentiated by the camera-coordinate Jacobian.
static void geometry_pose_step(double R[9], double t[3], const double dx[6]) {
    double x = dx[3], y = dx[4], z = dx[5];
    double th2 = x*x + y*y + z*z, th = sqrt(th2);
    double a = th < 1e-6 ? 1-th2/6 : sin(th)/th;
    double b = th < 1e-6 ? 0.5-th2/24 : (1-cos(th))/th2;
    double D[9] = {1-b*(y*y+z*z), b*x*y-a*z,     b*x*z+a*y,
                   b*x*y+a*z,     1-b*(x*x+z*z), b*y*z-a*x,
                   b*x*z-a*y,     b*y*z+a*x,     1-b*(x*x+y*y)};
    double Rn[9], tn[3];
    for (int r = 0; r < 3; r++) {
        tn[r] = D[3*r]*t[0] + D[3*r+1]*t[1] + D[3*r+2]*t[2] + dx[r];
        for (int c = 0; c < 3; c++)
            Rn[3*r+c] = D[3*r]*R[c] + D[3*r+1]*R[3+c] + D[3*r+2]*R[6+c];
    }
    memcpy(R, Rn, sizeof(Rn));
    memcpy(t, tn, sizeof(tn));
}

#endif
