// gcc -O2 -fopenmp tools/test_plus_geometry.c -lm -o /tmp/test_plus_geometry
#define main slam_main
#include "../simple_slam_c_plus.c"
#undef main
#include <assert.h>

static void check_svd(const double A[9]) {
    double s[3], U[9], V[9], scale = 0, err = 0;
    geometry_svd3(A, s, U, V);
    assert(s[0] >= s[1] && s[1] >= s[2] && s[2] >= 0);
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) {
        double a = 0, uu = 0, vv = 0;
        for (int k = 0; k < 3; k++) {
            a += U[3*r+k]*s[k]*V[3*c+k];
            uu += U[3*k+r]*U[3*k+c];
            vv += V[3*k+r]*V[3*k+c];
        }
        err = fmax(err, fabs(a-A[3*r+c]));
        scale = fmax(scale, fabs(A[3*r+c]));
        assert(fabs(uu-(r==c)) < 1e-12);
        assert(fabs(vv-(r==c)) < 1e-12);
    }
    assert(err <= 1e-12*fmax(scale, DBL_MIN));
}

static void check_eigen(int n) {
    double A[144] = {0}, B[144], W[12], V[144];
    for (int i = 0; i < n*n; i++) B[i] = sin(i*1.234+0.013*i*i+0.7);
    for (int r = 0; r < n; r++) for (int c = 0; c < n; c++)
        for (int k = 0; k < n; k++) A[r*n+c] += B[k*n+r]*B[k*n+c];
    geometry_eigen(A,n,W,V);
    for (int r = 0; r < n; r++) for (int c = 0; c < n; c++) {
        double av = 0;
        for (int k = 0; k < n; k++) av += A[r*n+k]*V[k*n+c];
        assert(fabs(av-W[c]*V[r*n+c]) < 1e-11);
    }
}

static void check_pose_jacobian(void) {
    double R[9] = {1,0,0, 0,1,0, 0,0,1}, t[3] = {20,-15,7};
    double P[3] = {-19,17,1}, cp[3] = {1,2,8};
    double J[2][6] = {
        {1./8,0,-1./64, -2./64,1+1./64,-2./8},
        {0,1./8,-2./64, -1-4./64,2./64,1./8}
    };
    for (int k = 0; k < 6; k++) {
        double Rn[9], tn[3], dx[6] = {0}, q[3];
        memcpy(Rn,R,sizeof(R)); memcpy(tn,t,sizeof(t));
        dx[k] = 1e-7;
        geometry_pose_step(Rn,tn,dx);
        for (int r = 0; r < 3; r++)
            q[r] = Rn[3*r]*P[0]+Rn[3*r+1]*P[1]+Rn[3*r+2]*P[2]+tn[r];
        for (int r = 0; r < 2; r++)
            assert(fabs((q[r]/q[2]-cp[r]/cp[2])/dx[k]-J[r][k]) < 1e-6);
    }
}

static void check_essential(void) {
    double R[9] = {1,0,0, 0,1,0, 0,0,1}, t[3] = {0.8,-0.2,0.1};
    double step[6] = {0,0,0, 0.05,-0.08,0.03};
    geometry_pose_step(R,t,step);
    double tx[9] = {0,-t[2],t[1], t[2],0,-t[0], -t[1],t[0],0}, E[9], orig[9];
    mat3_mul(tx,R,E); memcpy(orig,E,sizeof(E));
    enforce_essential_constraints(E);
    for (int i = 0; i < 9; i++) assert(fabs(E[i]-orig[i]) < 1e-12);
    Corner a[48], b[48]; Match m[48];
    for (int i = 0; i < 48; i++) {
        double P[3] = {((i*7)%17-8)*0.13, ((i*11)%19-9)*0.1, 3+(i%7)*0.43};
        double Q[3];
        for (int r = 0; r < 3; r++)
            Q[r] = R[3*r]*P[0]+R[3*r+1]*P[1]+R[3*r+2]*P[2]+t[r];
        a[i] = (Corner){500*P[0]/P[2]+320,500*P[1]/P[2]+240,-1,0,0};
        b[i] = (Corner){500*Q[0]/Q[2]+320,500*Q[1]/Q[2]+240,-1,0,0};
        m[i] = (Match){i,i,0};
    }
    CornerVec av = {a,48,48}, bv = {b,48,48}; MatchVec mv = {m,48,48};
    Pose pose; unsigned char *mask = NULL; int inliers = 0;
    assert(estimate_pose_E(&av,&bv,&mv,500,500,320,240,&pose,500,48,&mask,&inliers));
    assert(inliers == 48);
    double gotR[9], gott[3], norm = sqrt(vec3_dot(t,t));
    pose_get_rotation(&pose,gotR); pose_get_translation(&pose,gott);
    for (int i = 0; i < 9; i++) assert(fabs(gotR[i]-R[i]) < 1e-5);
    for (int i = 0; i < 3; i++) assert(fabs(gott[i]-t[i]/norm) < 1e-5);
    free(mask);
}

static void check_pnp(void) {
    // A negative camera translation and a distant world origin expose the
    // arbitrary-sign and conditioning errors in the old DLT candidate path.
    MapPoint points[80] = {0}; Corner corners[80];
    Map map = {points,80,80}; CornerVec cv = {corners,80,80};
    double R[9] = {1,0,0, 0,1,0, 0,0,1}, t[3] = {-1000,2000,-3000};
    for (int i = 0; i < 80; i++) {
        double x = ((i*7)%17-8)*0.13, y = ((i*11)%19-9)*0.1, z = 3+(i%7)*0.43;
        points[i] = (MapPoint){x-t[0],y-t[1],z-t[2],3,0,0,{{0}}};
        corners[i] = (Corner){500*x/z+320,500*y/z+240,i,0,0};
        if (i%5 == 0) { corners[i].x += 80; corners[i].y -= 65; }
    }
    Pose pose; int inliers;
    assert(estimate_pose_PnP(&map,&cv,500,500,320,240,500,2,&pose,&inliers));
    assert(inliers == 64);
    double gotR[9], gott[3];
    pose_get_rotation(&pose,gotR); pose_get_translation(&pose,gott);
    for (int i = 0; i < 9; i++) assert(fabs(gotR[i]-R[i]) < 1e-5);
    for (int i = 0; i < 3; i++) assert(fabs(gott[i]-t[i]) < 0.05);
    Corner clean[64]; int n = 0;
    for (int i = 0; i < 80; i++) if (i%5) clean[n++] = corners[i];
    CornerVec observations = {clean,64,64};
    double step[6] = {0.02,-0.01,0.03, 0.01,-0.015,0.02};
    geometry_pose_step(R,t,step);
    pose_from_rt(R,t,&pose);
    assert(pose_reprojection_rmse(&map,&observations,500,500,320,240,&pose) > 1);
    refine_pose_lm(&map,&observations,500,500,320,240,20,&pose);
    assert(pose_reprojection_rmse(&map,&observations,500,500,320,240,&pose) < 1e-4);
}

int main(void) {
    g_geometry_core = 1;
    check_svd((double[9]){0});
    check_svd((double[9]){1,2,3, 2,4,6, 3,6,9});
    check_svd((double[9]){0,-3,2, 3,0,-1, -2,1,0});
    for (int i = 0; i < 100; i++) {
        double A[9], scale = pow(10,(i%13-6)*20.0);
        for (int k = 0; k < 9; k++) A[k] = scale*sin(i*2.71+k*1.31+k*k*0.17);
        check_svd(A);
    }
    check_eigen(9); check_eigen(12);
    check_pose_jacobian(); check_essential(); check_pnp();
    puts("geometry: SVD, eigensystems, pose Jacobian, two-view recovery and PnP passed");
    return 0;
}
