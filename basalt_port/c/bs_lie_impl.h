/* Template body of bs_lie.c, included twice (S = float / double). Not a stand-alone header. */
#define BN(mod, name) BS_XCAT(bs_##mod, BS_XCAT(SFX, _##name))      /* bs_so3f_exp */
#define BJ(name) BS_XCAT(bs_##name, SFX)                            /* bs_right_jacobian_so3f */
#define Q BS_XCAT(bs_quat, SFX)
#define SO3 BS_XCAT(bs_so3, SFX)
#define SE3 BS_XCAT(bs_se3, SFX)
#define E3(name) BS_XCAT(bs_v3, BS_XCAT(SFX, _##name))
#define EM3(name) BS_XCAT(bs_m3, BS_XCAT(SFX, _##name))
#define EM6(name) BS_XCAT(bs_m6, BS_XCAT(SFX, _##name))
#define EQ(name) BS_XCAT(bs_q, BS_XCAT(SFX, _##name))
#define K(x) ((S)(x))

void BN(so3, identity)(SO3* o) { o->x = K(0); o->y = K(0); o->z = K(0); o->w = K(1); }

void BN(so3, normalize)(SO3* q) {
  S c[4] = {q->x, q->y, q->z, q->w};
  S length = SQRTF(EQ(sqn)(c));
  q->x /= length; q->y /= length; q->z /= length; q->w /= length;
}

void BN(so3, from_quat)(const Q* q, SO3* o) { *o = *q; BN(so3, normalize)(o); }

void BN(so3, exp)(const S w[3], SO3* o) {
  S theta_sq = E3(sqn)(w);
  S imag_factor, real_factor;
  if (theta_sq < EPS * EPS) {
    S theta_po4 = theta_sq * theta_sq;
    imag_factor = K(0.5) - K(1.0 / 48.0) * theta_sq + K(1.0 / 3840.0) * theta_po4;
    real_factor = K(1) - K(1.0 / 8.0) * theta_sq + K(1.0 / 384.0) * theta_po4;
  } else {
    S theta = SQRTF(theta_sq);
    S half_theta = K(0.5) * theta;
    S sin_half_theta = SINF(half_theta);
    imag_factor = sin_half_theta / theta;
    real_factor = COSF(half_theta);
  }
  o->w = real_factor; o->x = imag_factor * w[0]; o->y = imag_factor * w[1]; o->z = imag_factor * w[2];
}

void BN(so3, log)(const SO3* q, S o[3]) {
  S v[3] = {q->x, q->y, q->z};
  S squared_n = E3(sqn)(v);
  S w = q->w;
  S two_atan_nbyw_by_n;
  if (squared_n < EPS * EPS) {
    S squared_w = w * w;
    two_atan_nbyw_by_n = K(2) / w - K(2.0 / 3.0) * (squared_n) / (w * squared_w);
  } else {
    S n = SQRTF(squared_n);
    S atan_nbyw = (w < K(0)) ? K(ATAN2F(-n, -w)) : K(ATAN2F(n, w));
    two_atan_nbyw_by_n = K(2) * atan_nbyw / n;
  }
  o[0] = two_atan_nbyw_by_n * v[0]; o[1] = two_atan_nbyw_by_n * v[1]; o[2] = two_atan_nbyw_by_n * v[2];
}

void BN(so3, mul)(const SO3* a, const SO3* b, SO3* o) {
  Q r;
  r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
  r.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
  r.y = a->w * b->y + a->y * b->w + a->z * b->x - a->x * b->z;
  r.z = a->w * b->z + a->z * b->w + a->x * b->y - a->y * b->x;
  BN(so3, from_quat)(&r, o);
}

void BN(so3, inverse)(const SO3* q, SO3* o) {
  Q r; r.x = -q->x; r.y = -q->y; r.z = -q->z; r.w = q->w;
  BN(so3, from_quat)(&r, o);
}

void BN(so3, matrix)(const SO3* q, S o[9]) {
  const S tx = K(2) * q->x, ty = K(2) * q->y, tz = K(2) * q->z;
  const S twx = tx * q->w, twy = ty * q->w, twz = tz * q->w;
  const S txx = tx * q->x, txy = ty * q->x, txz = tz * q->x;
  const S tyy = ty * q->y, tyz = tz * q->y, tzz = tz * q->z;
  o[0] = K(1) - (tyy + tzz); o[3] = txy - twz;          o[6] = txz + twy;
  o[1] = txy + twz;          o[4] = K(1) - (txx + tzz); o[7] = tyz - twx;
  o[2] = txz - twy;          o[5] = tyz + twx;          o[8] = K(1) - (txx + tyy);
}

void BN(so3, act)(const SO3* q, const S p[3], S o[3]) {
  S v[3] = {q->x, q->y, q->z};
  S uv[3];
  E3(cross)(v, p, uv);
  uv[0] += uv[0]; uv[1] += uv[1]; uv[2] += uv[2];
  S c[3];
  E3(cross)(v, uv, c);
  S w = q->w;
  S r0 = (p[0] + w * uv[0]) + c[0];
  S r1 = (p[1] + w * uv[1]) + c[1];
  S r2 = (p[2] + w * uv[2]) + c[2];
  o[0] = r0; o[1] = r1; o[2] = r2;
}

void BN(so3, hat)(const S w[3], S o[9]) {
  o[0] = K(0); o[3] = -w[2]; o[6] = w[1];
  o[1] = w[2]; o[4] = K(0);  o[7] = -w[0];
  o[2] = -w[1]; o[5] = w[0]; o[8] = K(0);
}

void BN(so3, vee)(const S m[9], S o[3]) { o[0] = m[2 + 3 * 1]; o[1] = m[0 + 3 * 2]; o[2] = m[1 + 3 * 0]; }

/* ---- basalt-headers sophus_utils.hpp Jacobians ---- */
static void BS_XCAT(ident3_, SFX)(S J[9]) { for (int i = 0; i < 9; i++) J[i] = (i % 4 == 0) ? K(1) : K(0); }

void BJ(right_jacobian_so3)(const S phi[3], S J[9]) {
  S phi_norm2 = E3(sqn)(phi);
  S h[9], h2[9];
  BN(so3, hat)(phi, h);
  EM3(mul)(h, h, h2);
  BS_XCAT(ident3_, SFX)(J);
  if (phi_norm2 > EPS) {
    S phi_norm = SQRTF(phi_norm2);
    S phi_norm3 = phi_norm2 * phi_norm;
    S s1 = K(1) - COSF(phi_norm);
    for (int i = 0; i < 9; i++) J[i] = J[i] - (h[i] * s1) / phi_norm2;
    S s2 = phi_norm - SINF(phi_norm);
    for (int i = 0; i < 9; i++) J[i] = J[i] + (h2[i] * s2) / phi_norm3;
  } else {
    for (int i = 0; i < 9; i++) J[i] = J[i] - h[i] / K(2);
    for (int i = 0; i < 9; i++) J[i] = J[i] + h2[i] / K(6);
  }
}

void BJ(left_jacobian_so3)(const S phi[3], S J[9]) {
  S phi_norm2 = E3(sqn)(phi);
  S h[9], h2[9];
  BN(so3, hat)(phi, h);
  EM3(mul)(h, h, h2);
  BS_XCAT(ident3_, SFX)(J);
  if (phi_norm2 > EPS) {
    S phi_norm = SQRTF(phi_norm2);
    S phi_norm3 = phi_norm2 * phi_norm;
    S s1 = K(1) - COSF(phi_norm);
    for (int i = 0; i < 9; i++) J[i] = J[i] + (h[i] * s1) / phi_norm2;
    S s2 = phi_norm - SINF(phi_norm);
    for (int i = 0; i < 9; i++) J[i] = J[i] + (h2[i] * s2) / phi_norm3;
  } else {
    for (int i = 0; i < 9; i++) J[i] = J[i] + h[i] / K(2);
    for (int i = 0; i < 9; i++) J[i] = J[i] + h2[i] / K(6);
  }
}

/* shared tail of right/left inverse Jacobian: J already holds I +- phi_hat/2 */
static void BS_XCAT(inv_jac_tail_, SFX)(const S phi[3], const S h2[9], S phi_norm2, S J[9]) {
  (void)phi;
  if (phi_norm2 > EPS) {
    S phi_norm = SQRTF(phi_norm2);
    if ((double)phi_norm < BS_PI - (double)EPS_SQRT) {
      S sc = K(1) / phi_norm2 - (K(1) + COSF(phi_norm)) / (K(2) * phi_norm * SINF(phi_norm));
      for (int i = 0; i < 9; i++) J[i] = J[i] + h2[i] * sc;
    } else {
      for (int i = 0; i < 9; i++) J[i] = J[i] + h2[i] / PI2DIV;
    }
  } else {
    for (int i = 0; i < 9; i++) J[i] = J[i] + h2[i] / K(12);
  }
}

void BJ(right_jacobian_inv_so3)(const S phi[3], S J[9]) {
  S phi_norm2 = E3(sqn)(phi);
  S h[9], h2[9];
  BN(so3, hat)(phi, h);
  EM3(mul)(h, h, h2);
  BS_XCAT(ident3_, SFX)(J);
  for (int i = 0; i < 9; i++) J[i] = J[i] + h[i] / K(2);
  BS_XCAT(inv_jac_tail_, SFX)(phi, h2, phi_norm2, J);
}

void BJ(left_jacobian_inv_so3)(const S phi[3], S J[9]) {
  S phi_norm2 = E3(sqn)(phi);
  S h[9], h2[9];
  BN(so3, hat)(phi, h);
  EM3(mul)(h, h, h2);
  BS_XCAT(ident3_, SFX)(J);
  for (int i = 0; i < 9; i++) J[i] = J[i] - h[i] / K(2);
  BS_XCAT(inv_jac_tail_, SFX)(phi, h2, phi_norm2, J);
}

/* ---- SE3 ---- */
void BN(se3, identity)(SE3* o) { BN(so3, identity)(&o->so3); o->t[0] = o->t[1] = o->t[2] = K(0); }

void BN(se3, make)(const SO3* r, const S t[3], SE3* o) { o->so3 = *r; o->t[0] = t[0]; o->t[1] = t[1]; o->t[2] = t[2]; }

void BN(se3, act)(const SE3* a, const S p[3], S o[3]) {
  S r[3];
  BN(so3, act)(&a->so3, p, r);
  o[0] = r[0] + a->t[0]; o[1] = r[1] + a->t[1]; o[2] = r[2] + a->t[2];
}

void BN(se3, mul)(const SE3* a, const SE3* b, SE3* o) {
  SE3 r;
  BN(so3, mul)(&a->so3, &b->so3, &r.so3);
  S rt[3];
  BN(so3, act)(&a->so3, b->t, rt);
  r.t[0] = a->t[0] + rt[0]; r.t[1] = a->t[1] + rt[1]; r.t[2] = a->t[2] + rt[2];
  *o = r;
}

void BN(se3, inverse)(const SE3* a, SE3* o) {
  SE3 r;
  BN(so3, inverse)(&a->so3, &r.so3);
  S nt[3] = {a->t[0] * K(-1), a->t[1] * K(-1), a->t[2] * K(-1)};
  BN(so3, act)(&r.so3, nt, r.t);
  *o = r;
}

void BN(se3, matrix3x4)(const SE3* a, S o[12]) {
  BN(so3, matrix)(&a->so3, o);
  o[9] = a->t[0]; o[10] = a->t[1]; o[11] = a->t[2];
}

void BN(se3, matrix)(const SE3* a, S o[16]) {
  S m[12];
  BN(se3, matrix3x4)(a, m);
  for (int c = 0; c < 4; c++) { for (int r = 0; r < 3; r++) o[r + 4 * c] = m[r + 3 * c]; o[3 + 4 * c] = K(c == 3); }
}

void BN(se3, adj)(const SE3* a, S o[36]) {
  S R[9], h[9], hR[9], th[3] = {a->t[0], a->t[1], a->t[2]};
  BN(so3, matrix)(&a->so3, R);
  BN(so3, hat)(th, h);
  EM3(mul)(h, R, hR);
  for (int c = 0; c < 3; c++)
    for (int r = 0; r < 3; r++) {
      o[r + 6 * c] = R[r + 3 * c];                 /* (0,0) */
      o[(r + 3) + 6 * (c + 3)] = R[r + 3 * c];     /* (3,3) */
      o[r + 6 * (c + 3)] = hR[r + 3 * c];          /* (0,3) */
      o[(r + 3) + 6 * c] = K(0);                   /* (3,0) */
    }
}

void BJ(inc_pose)(const S inc[6], SE3* T) {
  T->t[0] += inc[0]; T->t[1] += inc[1]; T->t[2] += inc[2];
  SO3 e;
  BN(so3, exp)(inc + 3, &e);
  BN(so3, mul)(&e, &T->so3, &T->so3);
}

void BJ(compute_rel_pose)(const SE3* T_w_i_h, const SE3* T_i_c_h, const SE3* T_w_i_t, const SE3* T_i_c_t,
                          S* d_rel_d_h, S* d_rel_d_t, SE3* out) {
  SE3 tmp2;
  BN(se3, inverse)(T_i_c_t, &tmp2);
  SE3 T_t_i_h_i;
  SO3 inv_t;
  BN(so3, inverse)(&T_w_i_t->so3, &inv_t);
  BN(so3, mul)(&inv_t, &T_w_i_h->so3, &T_t_i_h_i.so3);
  S d[3] = {T_w_i_h->t[0] - T_w_i_t->t[0], T_w_i_h->t[1] - T_w_i_t->t[1], T_w_i_h->t[2] - T_w_i_t->t[2]};
  BN(so3, inverse)(&T_w_i_t->so3, &inv_t);
  BN(so3, act)(&inv_t, d, T_t_i_h_i.t);
  SE3 tmp, res;
  BN(se3, mul)(&tmp2, &T_t_i_h_i, &tmp);
  BN(se3, mul)(&tmp, T_i_c_h, &res);
  if (d_rel_d_h) {
    SO3 inv;
    S R[9], RR[36], A[36];
    BN(so3, inverse)(&T_w_i_h->so3, &inv);
    BN(so3, matrix)(&inv, R);
    for (int i = 0; i < 36; i++) RR[i] = K(0);
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++) { RR[r + 6 * c] = R[r + 3 * c]; RR[(r + 3) + 6 * (c + 3)] = R[r + 3 * c]; }
    BN(se3, adj)(&tmp, A);
    EM6(mul)(A, RR, d_rel_d_h);
  }
  if (d_rel_d_t) {
    SO3 inv;
    S R[9], RR[36], A[36];
    BN(so3, inverse)(&T_w_i_t->so3, &inv);
    BN(so3, matrix)(&inv, R);
    for (int i = 0; i < 36; i++) RR[i] = K(0);
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++) { RR[r + 6 * c] = R[r + 3 * c]; RR[(r + 3) + 6 * (c + 3)] = R[r + 3 * c]; }
    BN(se3, adj)(&tmp2, A);
    for (int i = 0; i < 36; i++) A[i] = -A[i];
    EM6(mul)(A, RR, d_rel_d_t);
  }
  *out = res;
}

int BN(quat, from_two_vectors)(const S a[3], const S b[3], Q* o) {
  S v0[3], v1[3];
  E3(normalized)(a, v0);
  E3(normalized)(b, v1);
  S c = E3(dot)(v1, v0);
  if (c < K(-1) + DUMMY_PREC) return 0;
  S axis[3];
  E3(cross)(v0, v1, axis);
  S s = SQRTF((K(1) + c) * K(2));
  S invs = K(1) / s;
  o->x = axis[0] * invs; o->y = axis[1] * invs; o->z = axis[2] * invs;
  o->w = s * K(0.5);
  return 1;
}
