/* SPDX-License-Identifier: MPL-2.0 */
/* Template body of bs_eigenf.c, included twice (S = float / double). Not a stand-alone header. */
#define BS_XCAT(a, b) BS_CAT(a, b)
#define BS_CAT(a, b) a##b

#define FN_(mod, name) BS_XCAT(bs_##mod, BS_XCAT(SFX, _##name))

S FN_(v3, sqn)(const S v[3]) {
#if BS_LEFT3
  return (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
#else
  return v[0] * v[0] + (v[1] * v[1] + v[2] * v[2]);
#endif
}

S FN_(v3, dot)(const S a[3], const S b[3]) {
#if BS_LEFT3
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
#else
  return a[0] * b[0] + (a[1] * b[1] + a[2] * b[2]);
#endif
}

void FN_(v3, cross)(const S a[3], const S b[3], S out[3]) {
  S r0 = a[1] * b[2] - a[2] * b[1];
  S r1 = a[2] * b[0] - a[0] * b[2];
  S r2 = a[0] * b[1] - a[1] * b[0];
  out[0] = r0; out[1] = r1; out[2] = r2;
}

int FN_(v3, normalized)(const S v[3], S out[3]) {
  S z = FN_(v3, sqn)(v);
  if (z > (S)0) {
    S n = SQRTF(z);
    S r0 = v[0] / n, r1 = v[1] / n, r2 = v[2] / n;
    out[0] = r0; out[1] = r1; out[2] = r2;
    return 1;
  }
  out[0] = v[0]; out[1] = v[1]; out[2] = v[2];
  return 0;
}

S FN_(q, sqn)(const S q[4]) {
  return (q[0] * q[0] + q[2] * q[2]) + (q[1] * q[1] + q[3] * q[3]);
}

void FN_(m3, mul)(const S a[9], const S b[9], S out[9]) {
  S r[9];
  for (int c = 0; c < 3; c++)
    for (int i = 0; i < 3; i++) {
      S p0 = a[i + 0] * b[0 + 3 * c], p1 = a[i + 3] * b[1 + 3 * c], p2 = a[i + 6] * b[2 + 3 * c];
#if BS_LEFT3
      r[i + 3 * c] = (i < 2) ? (p0 + p1) + p2 : p0 + (p1 + p2);   /* double: rows 0-1 packet, row 2 scalar */
#else
      r[i + 3 * c] = p0 + (p1 + p2);
#endif
    }
  for (int i = 0; i < 9; i++) out[i] = r[i];
}

void FN_(m3, mulv)(const S a[9], const S v[3], S out[3]) {
  S r[3];
  for (int i = 0; i < 3; i++) {
    S p0 = a[i + 0] * v[0], p1 = a[i + 3] * v[1], p2 = a[i + 6] * v[2];
#if BS_LEFT3
    r[i] = (i < 2) ? (p0 + p1) + p2 : p0 + (p1 + p2);
#else
    r[i] = p0 + (p1 + p2);
#endif
  }
  out[0] = r[0]; out[1] = r[1]; out[2] = r[2];
}

void FN_(m3, transpose)(const S a[9], S out[9]) {
  S r[9];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) r[i + 3 * j] = a[j + 3 * i];
  for (int i = 0; i < 9; i++) out[i] = r[i];
}

void FN_(m6, mul)(const S a[36], const S b[36], S out[36]) {
  S r[36];
  for (int c = 0; c < 6; c++) {
#if BS_PACKET4
    int start = 0, pend = 4;               /* rows 0-3 packet, rows 4-5 scalar tail, every column */
#else
    int start = 0, pend = 6;               /* double: 6 rows = 3 packets of 2 */
#endif
    for (int i = 0; i < 6; i++) {
      S p[6];
      for (int k = 0; k < 6; k++) p[k] = a[i + 6 * k] * b[k + 6 * c];
      if (i >= start && i < pend) {
        S s = p[0];
        for (int k = 1; k < 6; k++) s = s + p[k];
        r[i + 6 * c] = s;
      } else {
        r[i + 6 * c] = (p[0] + (p[1] + p[2])) + (p[3] + (p[4] + p[5]));
      }
    }
  }
  for (int i = 0; i < 36; i++) out[i] = r[i];
}
