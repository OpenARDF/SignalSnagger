#include <math.h>
#include <stdint.h>
void draw(int w, int h, int n, const float *p, const uint8_t *col, uint8_t *rgb,
          float *buf, int16_t *tags, const int16_t *inst, float *normals,
          const float *flat) {
  for (int i = 0; i < n; i++) {
    const float *a = p + 9 * i, *b = a + 3, *c = b + 3;
    float det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
    if (fabsf(det) < .0001f)
      continue;
    int x0 = fmaxf(0, floorf(fminf(a[0], fminf(b[0], c[0])))),
        x1 = fminf(w - 1, ceilf(fmaxf(a[0], fmaxf(b[0], c[0]))));
    int y0 = fmaxf(0, floorf(fminf(a[1], fminf(b[1], c[1])))),
        y1 = fminf(h - 1, ceilf(fmaxf(a[1], fmaxf(b[1], c[1]))));
    for (int y = y0; y <= y1; y++)
      for (int x = x0; x <= x1; x++) {
        float ex = x + .5f - c[0], ey = y + .5f - c[1],
              u = ((b[1] - c[1]) * ex + (c[0] - b[0]) * ey) / det,
              v = ((c[1] - a[1]) * ex + (a[0] - c[0]) * ey) / det,
              q = 1 - u - v;
        if (u < -.00001f || v < -.00001f || q < -.00001f)
          continue;
        float z = u * a[2] + v * b[2] + q * c[2];
        int k = y * w + x;
        if (z < buf[k])
          continue;
        buf[k] = z;
        tags[k] = inst[i];
        for (int j = 0; j < 3; j++) {
          rgb[3 * k + j] = col[3 * i + j];
          normals[3 * k + j] = flat[3 * i + j];
        }
      }
  }
}
