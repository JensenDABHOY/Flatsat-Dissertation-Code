/*
 * spectral_proc.c  (fixed build + consistent function signatures)
 *
 * Build (Pi):
 *   gcc -O3 -std=c11 spectral_proc.c -lm -o spectral_proc
 *
 * If you see clock_gettime issues:
 *   gcc -O3 -std=c11 -D_POSIX_C_SOURCE=200809L spectral_proc.c -lm -o spectral_proc
 *
 * Run:
 *   ./spectral_proc --frame frame.bin --rows 64 --cols 2048 --wl-min 300 --wl-max 330 --json
 */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEFAULT_INST_FWHM_NM 0.25
#define DEFAULT_MOFFAT_BETA 2.5

typedef struct {
    double centroid_nm;
    double fwhm_nm;
    double alpha_nm;
    double beta;
    double peak_height_adu;
    double integrated_flux;
    double snr;
    int matched_index;
    double matched_wavelength_nm;
} LineFitResult;

typedef struct {
    double wavelength_nm;
} LineCatalogEntry;

/* ---------- prototypes (prevents signature mismatch / implicit decl) ---------- */

static inline double moffat_profile(double x, double amp, double center, double alpha, double beta);
static inline double moffat_fwhm(double alpha, double beta);
static inline double moffat_integrated_flux(double amp, double alpha, double beta);

static inline size_t packet_bytes(int line_count);
static inline double packet_kb(int line_count);

static double now_ms(void);

static int cmp_double(const void *a, const void *b);
static double median_double(double *buf, int n);
static double mad_double(const double *values, int n);

static int solve_4x4(double A[4][4], double b[4], double x[4]);

static void polyfit3(const double *x, const double *y, const uint8_t *mask, int n, double coeffs[4]);
static double polyval3(double x, const double coeffs[4]);

int fit_continuum_poly3(
    const double *wavelengths_nm,
    const double *spectrum_adu,
    int n,
    int n_iter,
    double sigma_clip,
    double *continuum_out
);

int flatten_spectrum(
    const double *wavelengths_nm,
    const double *spectrum_adu,
    int n,
    int n_iter,
    double sigma_clip,
    double *continuum_out,
    double *line_only_out,
    double *noise_std_out
);

int detect_peaks(
    const double *y,
    int n,
    double threshold,
    int min_distance_px,
    int *out_idx,
    int max_out
);

static int lm_fit_moffat(
    const double *x,
    const double *y,
    int n,
    double *p,
    const double *lower,
    const double *upper,
    int max_iter
);

void extract_1d_spectrum_sum(
    const double *frame_adu,
    int n_rows,
    int n_cols,
    double *spectrum_out
);

int fit_moffat_lines(
    const double *wavelengths_nm,
    const double *line_only_adu,
    int n,
    double noise_std,
    double snr_min,
    double fit_window_nm,
    double inst_fwhm_nm,
    double beta_guess,
    const LineCatalogEntry *catalog,
    int catalog_count,
    double match_tolerance_nm,
    LineFitResult *out,
    int out_capacity
);

/* IMPORTANT: signature must match ALL calls */
int process_spectrum(
    const double *wavelengths_nm,
    const double *spectrum_adu,
    int n,
    int n_iter,
    double sigma_clip,
    double snr_min,
    double fit_window_nm,
    double inst_fwhm_nm,
    double beta_guess,
    const LineCatalogEntry *catalog,
    int catalog_count,
    double match_tolerance_nm,
    double *continuum_out,
    double *line_only_out,
    double *noise_std_out,
    LineFitResult *out,
    int out_capacity
);

int process_frame(
    const double *frame_adu,
    int n_rows,
    int n_cols,
    const double *wavelengths_nm,
    int n_iter,
    double sigma_clip,
    double snr_min,
    double fit_window_nm,
    double inst_fwhm_nm,
    double beta_guess,
    const LineCatalogEntry *catalog,
    int catalog_count,
    double match_tolerance_nm,
    double *spectrum_out,
    double *continuum_out,
    double *line_only_out,
    double *noise_std_out,
    LineFitResult *out,
    int out_capacity
);

/* ---------- moffat + packet ---------- */

static inline double moffat_profile(double x, double amp, double center, double alpha, double beta) {
    double t = (x - center) / alpha;
    double base = 1.0 + t * t;
    return amp * pow(base, -beta);
}

static inline double moffat_fwhm(double alpha, double beta) {
    return 2.0 * alpha * sqrt(pow(2.0, 1.0 / beta) - 1.0);
}

static inline double moffat_integrated_flux(double amp, double alpha, double beta) {
    return amp * alpha * sqrt(M_PI) * tgamma(beta - 0.5) / tgamma(beta);
}

/* Packet model: 2 bytes header + N lines * 4 fields * 4 bytes */
static inline size_t packet_bytes(int line_count) {
    const size_t header_bytes = 2;
    const size_t fields_per_line = 4;
    const size_t bytes_per_field = 4;
    if (line_count < 0) line_count = 0;
    return header_bytes + (size_t)line_count * fields_per_line * bytes_per_field;
}

static inline double packet_kb(int line_count) {
    return (double)packet_bytes(line_count) / 1024.0;
}

static double now_ms(void) {
#if defined(CLOCK_MONOTONIC)
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        return 1000.0 * (double)ts.tv_sec + 1e-6 * (double)ts.tv_nsec;
    }
#endif
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return 1000.0 * (double)tv.tv_sec + 1e-3 * (double)tv.tv_usec;
}

/* ---------- robust stats ---------- */

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    return (da > db) - (da < db);
}

static double median_double(double *buf, int n) {
    qsort(buf, (size_t)n, sizeof(double), cmp_double);
    if (n % 2) return buf[n / 2];
    return 0.5 * (buf[n / 2 - 1] + buf[n / 2]);
}

static double mad_double(const double *values, int n) {
    if (n <= 0) return NAN;

    double *tmp = (double *)malloc((size_t)n * sizeof(double));
    double *dev = (double *)malloc((size_t)n * sizeof(double));
    if (!tmp || !dev) {
        free(tmp);
        free(dev);
        return NAN;
    }

    memcpy(tmp, values, (size_t)n * sizeof(double));
    double med = median_double(tmp, n);

    for (int i = 0; i < n; i++) dev[i] = fabs(values[i] - med);

    double mad = median_double(dev, n);
    free(tmp);
    free(dev);
    return mad;
}

/* ---------- linear algebra ---------- */

static int solve_4x4(double A[4][4], double b[4], double x[4]) {
    double M[4][5];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) M[i][j] = A[i][j];
        M[i][4] = b[i];
    }

    for (int i = 0; i < 4; i++) {
        int pivot = i;
        double maxv = fabs(M[i][i]);
        for (int r = i + 1; r < 4; r++) {
            if (fabs(M[r][i]) > maxv) {
                maxv = fabs(M[r][i]);
                pivot = r;
            }
        }
        if (maxv < 1e-12) return 0;

        if (pivot != i) {
            for (int c = i; c < 5; c++) {
                double tmp = M[i][c];
                M[i][c] = M[pivot][c];
                M[pivot][c] = tmp;
            }
        }

        double inv = 1.0 / M[i][i];
        for (int c = i; c < 5; c++) M[i][c] *= inv;

        for (int r = 0; r < 4; r++) {
            if (r == i) continue;
            double factor = M[r][i];
            for (int c = i; c < 5; c++) M[r][c] -= factor * M[i][c];
        }
    }

    for (int i = 0; i < 4; i++) x[i] = M[i][4];
    return 1;
}

/* ---------- continuum fit ---------- */

static void polyfit3(const double *x, const double *y, const uint8_t *mask, int n, double coeffs[4]) {
    double S0 = 0.0, S1 = 0.0, S2 = 0.0, S3 = 0.0, S4 = 0.0, S5 = 0.0, S6 = 0.0;
    double Y0 = 0.0, Y1 = 0.0, Y2 = 0.0, Y3 = 0.0;

    for (int i = 0; i < n; i++) {
        if (!mask[i]) continue;
        double xi = x[i];
        double x2 = xi * xi;
        double x3 = x2 * xi;
        double x4 = x2 * x2;
        double x5 = x4 * xi;
        double x6 = x3 * x3;

        S0 += 1.0;
        S1 += xi;
        S2 += x2;
        S3 += x3;
        S4 += x4;
        S5 += x5;
        S6 += x6;

        Y0 += y[i];
        Y1 += y[i] * xi;
        Y2 += y[i] * x2;
        Y3 += y[i] * x3;
    }

    double A[4][4] = {
        {S0, S1, S2, S3},
        {S1, S2, S3, S4},
        {S2, S3, S4, S5},
        {S3, S4, S5, S6}
    };
    double b[4] = {Y0, Y1, Y2, Y3};
    double out[4];

    if (!solve_4x4(A, b, out)) {
        coeffs[0] = coeffs[1] = coeffs[2] = coeffs[3] = 0.0;
        return;
    }

    coeffs[0] = out[0];
    coeffs[1] = out[1];
    coeffs[2] = out[2];
    coeffs[3] = out[3];
}

static double polyval3(double x, const double coeffs[4]) {
    return ((coeffs[3] * x + coeffs[2]) * x + coeffs[1]) * x + coeffs[0];
}

int fit_continuum_poly3(
    const double *wavelengths_nm,
    const double *spectrum_adu,
    int n,
    int n_iter,
    double sigma_clip,
    double *continuum_out
) {
    double min_w = wavelengths_nm[0];
    double max_w = wavelengths_nm[0];
    for (int i = 1; i < n; i++) {
        if (wavelengths_nm[i] < min_w) min_w = wavelengths_nm[i];
        if (wavelengths_nm[i] > max_w) max_w = wavelengths_nm[i];
    }
    double mean_w = 0.5 * (min_w + max_w);
    double span = (max_w - min_w);
    if (span <= 0.0) return 0;

    double *x = (double *)malloc((size_t)n * sizeof(double));
    uint8_t *mask = (uint8_t *)malloc((size_t)n);
    double *resid = (double *)malloc((size_t)n * sizeof(double));
    if (!x || !mask || !resid) {
        free(x); free(mask); free(resid);
        return 0;
    }

    for (int i = 0; i < n; i++) {
        x[i] = (wavelengths_nm[i] - mean_w) / span;
        mask[i] = 1u;
    }

    double coeffs[4] = {0};

    for (int iter = 0; iter < n_iter; iter++) {
        polyfit3(x, spectrum_adu, mask, n, coeffs);

        int count = 0;
        for (int i = 0; i < n; i++) {
            if (!mask[i]) continue;
            resid[count++] = spectrum_adu[i] - polyval3(x[i], coeffs);
        }
        if (count <= 8) break;

        double mad = mad_double(resid, count);
        double scale = (mad > 0.0) ? (1.4826 * mad) : 0.0;
        if (scale <= 0.0) break;

        for (int i = 0; i < n; i++) {
            double r = spectrum_adu[i] - polyval3(x[i], coeffs);
            mask[i] = (fabs(r) <= sigma_clip * scale) ? 1u : 0u;
        }
    }

    for (int i = 0; i < n; i++) continuum_out[i] = polyval3(x[i], coeffs);

    free(x);
    free(mask);
    free(resid);
    return 1;
}

int flatten_spectrum(
    const double *wavelengths_nm,
    const double *spectrum_adu,
    int n,
    int n_iter,
    double sigma_clip,
    double *continuum_out,
    double *line_only_out,
    double *noise_std_out
) {
    if (!fit_continuum_poly3(wavelengths_nm, spectrum_adu, n, n_iter, sigma_clip, continuum_out)) return 0;

    for (int i = 0; i < n; i++) line_only_out[i] = spectrum_adu[i] - continuum_out[i];

    double mad = mad_double(line_only_out, n);
    double noise_std = (mad > 0.0) ? (1.4826 * mad) : 0.0;

    if (!(noise_std > 0.0)) {
        double sum = 0.0;
        for (int i = 0; i < n; i++) sum += line_only_out[i] * line_only_out[i];
        noise_std = sqrt(sum / (double)n);
    }

    *noise_std_out = noise_std;
    return 1;
}

/* ---------- peak detection + moffat fitting ---------- */

int detect_peaks(
    const double *y,
    int n,
    double threshold,
    int min_distance_px,
    int *out_idx,
    int max_out
) {
    int count = 0;
    int last_idx = -min_distance_px;

    for (int i = 1; i < n - 1; i++) {
        if (y[i] < threshold) continue;
        if (y[i] >= y[i - 1] && y[i] >= y[i + 1]) {
            if (i - last_idx >= min_distance_px) {
                if (count < max_out) {
                    out_idx[count++] = i;
                    last_idx = i;
                }
            } else if (count > 0 && y[i] > y[last_idx]) {
                out_idx[count - 1] = i;
                last_idx = i;
            }
        }
    }
    return count;
}

static int lm_fit_moffat(
    const double *x,
    const double *y,
    int n,
    double *p,
    const double *lower,
    const double *upper,
    int max_iter
) {
    double lambda = 1e-3;
    double cost = 0.0;

    for (int i = 0; i < n; i++) {
        double r = y[i] - moffat_profile(x[i], p[0], p[1], p[2], p[3]);
        cost += r * r;
    }

    for (int iter = 0; iter < max_iter; iter++) {
        double JTJ[4][4] = {{0}};
        double JTr[4] = {0, 0, 0, 0};

        for (int i = 0; i < n; i++) {
            double dx = (x[i] - p[1]);
            double inv_alpha = 1.0 / p[2];
            double t = dx * inv_alpha;
            double base = 1.0 + t * t;
            double base_beta = pow(base, -p[3]);
            double model = p[0] * base_beta;
            double r = y[i] - model;

            double j_amp = base_beta;
            double j_center = p[0] * p[3] * 2.0 * dx * inv_alpha * inv_alpha * pow(base, -p[3] - 1.0);
            double j_alpha  = p[0] * p[3] * 2.0 * dx * dx * inv_alpha * inv_alpha * inv_alpha * pow(base, -p[3] - 1.0);
            double j_beta   = -p[0] * base_beta * log(base);

            double J[4] = {j_amp, j_center, j_alpha, j_beta};

            for (int a = 0; a < 4; a++) {
                JTr[a] += J[a] * r;
                for (int b = a; b < 4; b++) JTJ[a][b] += J[a] * J[b];
            }
        }

        for (int a = 0; a < 4; a++) {
            for (int b = a + 1; b < 4; b++) JTJ[b][a] = JTJ[a][b];
            JTJ[a][a] += lambda;
        }

        double delta[4] = {0, 0, 0, 0};
        if (!solve_4x4(JTJ, JTr, delta)) return 0;

        double p_new[4] = {p[0] + delta[0], p[1] + delta[1], p[2] + delta[2], p[3] + delta[3]};
        for (int k = 0; k < 4; k++) {
            if (p_new[k] < lower[k]) p_new[k] = lower[k];
            if (p_new[k] > upper[k]) p_new[k] = upper[k];
        }

        double cost_new = 0.0;
        for (int i = 0; i < n; i++) {
            double r = y[i] - moffat_profile(x[i], p_new[0], p_new[1], p_new[2], p_new[3]);
            cost_new += r * r;
        }

        if (cost_new < cost) {
            p[0] = p_new[0];
            p[1] = p_new[1];
            p[2] = p_new[2];
            p[3] = p_new[3];
            cost = cost_new;
            lambda *= 0.7;
        } else {
            lambda *= 2.0;
        }

        double dn = sqrt(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3]);
        if (dn < 1e-6) break;
    }

    return 1;
}

void extract_1d_spectrum_sum(
    const double *frame_adu,
    int n_rows,
    int n_cols,
    double *spectrum_out
) {
    for (int c = 0; c < n_cols; c++) {
        double acc = 0.0;
        for (int r = 0; r < n_rows; r++) acc += frame_adu[r * n_cols + c];
        spectrum_out[c] = acc;
    }
}

int fit_moffat_lines(
    const double *wavelengths_nm,
    const double *line_only_adu,
    int n,
    double noise_std,
    double snr_min,
    double fit_window_nm,
    double inst_fwhm_nm,
    double beta_guess,
    const LineCatalogEntry *catalog,
    int catalog_count,
    double match_tolerance_nm,
    LineFitResult *out,
    int out_capacity
) {
    double pixel_scale = (wavelengths_nm[n - 1] - wavelengths_nm[0]) / (double)(n - 1);
    if (!(pixel_scale > 0.0)) return 0;

    int min_distance = (int)fmax(1.0, inst_fwhm_nm / pixel_scale);
    double threshold = snr_min * noise_std;

    int *peak_idx = (int *)malloc((size_t)n * sizeof(int));
    if (!peak_idx) return 0;

    int peak_count = detect_peaks(line_only_adu, n, threshold, min_distance, peak_idx, n);

    int max_win = (int)(2.0 * fit_window_nm / pixel_scale) + 16;
    if (max_win < 16) max_win = 16;
    if (max_win > n) max_win = n;

    double *xbuf = (double *)malloc((size_t)max_win * sizeof(double));
    double *ybuf = (double *)malloc((size_t)max_win * sizeof(double));
    if (!xbuf || !ybuf) {
        free(peak_idx);
        free(xbuf); free(ybuf);
        return 0;
    }

    int out_count = 0;

    for (int pidx = 0; pidx < peak_count && out_count < out_capacity; pidx++) {
        int idx = peak_idx[pidx];
        double center_guess = wavelengths_nm[idx];
        double amp_guess = line_only_adu[idx];

        double left = center_guess - fit_window_nm;
        double right = center_guess + fit_window_nm;

        int start = 0;
        int end = n - 1;
        while (start < n && wavelengths_nm[start] < left) start++;
        while (end > 0 && wavelengths_nm[end] > right) end--;
        int count = end - start + 1;
        if (count < 6) continue;
        if (count > max_win) count = max_win;

        for (int i = 0; i < count; i++) {
            xbuf[i] = wavelengths_nm[start + i];
            ybuf[i] = line_only_adu[start + i];
        }

        double alpha_guess = inst_fwhm_nm / (2.0 * sqrt(pow(2.0, 1.0 / beta_guess) - 1.0));
        double params[4] = {amp_guess, center_guess, alpha_guess, beta_guess};
        double lower[4] = {0.0, center_guess - fit_window_nm, 1e-4, 1.1};
        double upper[4] = {1e30, center_guess + fit_window_nm, 5.0, 10.0};

        if (!lm_fit_moffat(xbuf, ybuf, count, params, lower, upper, 30)) continue;

        LineFitResult *res = &out[out_count++];
        res->peak_height_adu = params[0];
        res->centroid_nm = params[1];
        res->alpha_nm = params[2];
        res->beta = params[3];
        res->fwhm_nm = moffat_fwhm(params[2], params[3]);
        res->integrated_flux = moffat_integrated_flux(params[0], params[2], params[3]);
        res->snr = (noise_std > 0.0) ? (params[0] / noise_std) : 0.0;
        res->matched_index = -1;
        res->matched_wavelength_nm = 0.0;

        if (catalog && catalog_count > 0) {
            double best = 1e30;
            int best_idx = -1;
            for (int j = 0; j < catalog_count; j++) {
                double d = fabs(res->centroid_nm - catalog[j].wavelength_nm);
                if (d < best) {
                    best = d;
                    best_idx = j;
                }
            }
            if (best <= match_tolerance_nm) {
                res->matched_index = best_idx;
                res->matched_wavelength_nm = catalog[best_idx].wavelength_nm;
            }
        }
    }

    free(xbuf);
    free(ybuf);
    free(peak_idx);
    return out_count;
}

/* ---------- top-level processing (FIXED ordering + args) ---------- */

int process_spectrum(
    const double *wavelengths_nm,
    const double *spectrum_adu,
    int n,
    int n_iter,
    double sigma_clip,
    double snr_min,
    double fit_window_nm,
    double inst_fwhm_nm,
    double beta_guess,
    const LineCatalogEntry *catalog,
    int catalog_count,
    double match_tolerance_nm,
    double *continuum_out,
    double *line_only_out,
    double *noise_std_out,
    LineFitResult *out,
    int out_capacity
) {
    if (!flatten_spectrum(wavelengths_nm, spectrum_adu, n, n_iter, sigma_clip,
                         continuum_out, line_only_out, noise_std_out)) {
        return 0;
    }

    return fit_moffat_lines(
        wavelengths_nm,
        line_only_out,
        n,
        *noise_std_out,
        snr_min,
        fit_window_nm,
        inst_fwhm_nm,
        beta_guess,
        catalog,
        catalog_count,
        match_tolerance_nm,
        out,
        out_capacity
    );
}

int process_frame(
    const double *frame_adu,
    int n_rows,
    int n_cols,
    const double *wavelengths_nm,
    int n_iter,
    double sigma_clip,
    double snr_min,
    double fit_window_nm,
    double inst_fwhm_nm,
    double beta_guess,
    const LineCatalogEntry *catalog,
    int catalog_count,
    double match_tolerance_nm,
    double *spectrum_out,
    double *continuum_out,
    double *line_only_out,
    double *noise_std_out,
    LineFitResult *out,
    int out_capacity
) {
    extract_1d_spectrum_sum(frame_adu, n_rows, n_cols, spectrum_out);

    /* FIX: call matches process_spectrum signature exactly */
    return process_spectrum(
        wavelengths_nm,
        spectrum_out,
        n_cols,
        n_iter,
        sigma_clip,
        snr_min,
        fit_window_nm,
        inst_fwhm_nm,
        beta_guess,
        catalog,
        catalog_count,
        match_tolerance_nm,
        continuum_out,
        line_only_out,
        noise_std_out,
        out,
        out_capacity
    );
}

/* ---------- CLI parsing ---------- */

static int argi(int *i, int argc, char **argv) {
    if (*i + 1 >= argc) return 0;
    (*i)++;
    return 1;
}

static void usage(void) {
    printf("spectral_proc\n");
    printf("  --frame <path> (float64 doubles row-major)\n");
    printf("  --rows <int>   default 64\n");
    printf("  --cols <int>   default 2048\n");
    printf("  --wl-min <nm>  default 300\n");
    printf("  --wl-max <nm>  default 330\n");
    printf("  --n-iter <int> default 5\n");
    printf("  --sigma-clip <double> default 3\n");
    printf("  --snr-min <double> default 5\n");
    printf("  --fit-window <nm> default 0.6\n");
    printf("  --inst-fwhm <nm> default 0.25\n");
    printf("  --beta <double> default 2.5\n");
    printf("  --match-tol <nm> default 0.15\n");
    printf("  --out-cap <int> default 64\n");
    printf("  --json (print JSON line)\n");
}

int main(int argc, char **argv) {
    const char *frame_path = NULL;
    int rows = 64;
    int cols = 2048;
    double wl_min = 300.0;
    double wl_max = 330.0;
    int n_iter = 5;
    double sigma_clip = 3.0;
    double snr_min = 5.0;
    double fit_window_nm = 0.6;
    double inst_fwhm_nm = DEFAULT_INST_FWHM_NM;
    double beta_guess = DEFAULT_MOFFAT_BETA;
    double match_tol = 0.15;
    int out_cap = 64;
    int want_json = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--frame")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            frame_path = argv[i];
        } else if (!strcmp(argv[i], "--rows")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            rows = atoi(argv[i]);
        } else if (!strcmp(argv[i], "--cols")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            cols = atoi(argv[i]);
        } else if (!strcmp(argv[i], "--wl-min")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            wl_min = atof(argv[i]);
        } else if (!strcmp(argv[i], "--wl-max")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            wl_max = atof(argv[i]);
        } else if (!strcmp(argv[i], "--n-iter")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            n_iter = atoi(argv[i]);
        } else if (!strcmp(argv[i], "--sigma-clip")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            sigma_clip = atof(argv[i]);
        } else if (!strcmp(argv[i], "--snr-min")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            snr_min = atof(argv[i]);
        } else if (!strcmp(argv[i], "--fit-window")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            fit_window_nm = atof(argv[i]);
        } else if (!strcmp(argv[i], "--inst-fwhm")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            inst_fwhm_nm = atof(argv[i]);
        } else if (!strcmp(argv[i], "--beta")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            beta_guess = atof(argv[i]);
        } else if (!strcmp(argv[i], "--match-tol")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            match_tol = atof(argv[i]);
        } else if (!strcmp(argv[i], "--out-cap")) {
            if (!argi(&i, argc, argv)) { usage(); return 2; }
            out_cap = atoi(argv[i]);
        } else if (!strcmp(argv[i], "--json")) {
            want_json = 1;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            usage();
            return 0;
        } else {
            printf("Unknown arg %s\n", argv[i]);
            usage();
            return 2;
        }
    }

    if (!frame_path) { printf("Missing --frame\n"); usage(); return 2; }
    if (rows <= 0 || cols <= 0) { printf("Bad rows/cols\n"); return 2; }
    if (!(wl_max > wl_min)) { printf("Bad wl range\n"); return 2; }

    size_t n_frame = (size_t)rows * (size_t)cols;

    FILE *f = fopen(frame_path, "rb");
    if (!f) { printf("Could not open %s\n", frame_path); return 1; }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    long expect = (long)(n_frame * sizeof(double));
    if (fsize != expect) {
        printf("Frame size mismatch. got=%ld expected=%ld bytes\n", fsize, expect);
        fclose(f);
        return 1;
    }

    double *frame = (double *)malloc(n_frame * sizeof(double));
    double *wavelengths = (double *)malloc((size_t)cols * sizeof(double));
    double *spectrum = (double *)malloc((size_t)cols * sizeof(double));
    double *continuum = (double *)malloc((size_t)cols * sizeof(double));
    double *line_only = (double *)malloc((size_t)cols * sizeof(double));
    LineFitResult *results = (LineFitResult *)malloc((size_t)out_cap * sizeof(LineFitResult));

    if (!frame || !wavelengths || !spectrum || !continuum || !line_only || !results) {
        printf("Allocation failed\n");
        fclose(f);
        free(frame); free(wavelengths); free(spectrum); free(continuum); free(line_only); free(results);
        return 1;
    }

    size_t nread = fread(frame, sizeof(double), n_frame, f);
    fclose(f);
    if (nread != n_frame) {
        printf("Read failed\n");
        free(frame); free(wavelengths); free(spectrum); free(continuum); free(line_only); free(results);
        return 1;
    }

    for (int i = 0; i < cols; i++) {
        wavelengths[i] = wl_min + (wl_max - wl_min) * (double)i / (double)(cols - 1);
    }

    double noise_std = 0.0;
    double t0 = now_ms();

    int line_count = process_frame(
        frame, rows, cols, wavelengths,
        n_iter, sigma_clip, snr_min,
        fit_window_nm, inst_fwhm_nm, beta_guess,
        NULL, 0, match_tol,
        spectrum, continuum, line_only, &noise_std,
        results, out_cap
    );

    double t1 = now_ms();

    double time_ms = t1 - t0;
    size_t pbytes = packet_bytes(line_count);

    printf("LINES=%d NOISE_STD=%.6f TIME_MS=%.3f PACKET_BYTES=%zu PACKET_KB=%.3f\n",
           line_count, noise_std, time_ms, pbytes, packet_kb(line_count));

    for (int i = 0; i < line_count; i++) {
        LineFitResult *r = &results[i];
        printf("LINE %d CENTROID_NM=%.6f FWHM_NM=%.6f PEAK=%.3f FLUX=%.3f SNR=%.3f ALPHA=%.6f BETA=%.6f\n",
               i, r->centroid_nm, r->fwhm_nm, r->peak_height_adu, r->integrated_flux, r->snr, r->alpha_nm, r->beta);
    }

    if (want_json) {
        printf("JSON:{\"line_count\":%d,\"noise_std\":%.10f,\"time_ms\":%.6f,\"packet_bytes\":%zu,\"lines\":[",
               line_count, noise_std, time_ms, pbytes);
        for (int i = 0; i < line_count; i++) {
            LineFitResult *r = &results[i];
            printf("{\"i\":%d,\"centroid_nm\":%.10f,\"fwhm_nm\":%.10f,\"peak\":%.10f,\"flux\":%.10f,\"snr\":%.10f}",
                   i, r->centroid_nm, r->fwhm_nm, r->peak_height_adu, r->integrated_flux, r->snr);
            if (i + 1 < line_count) printf(",");
        }
        printf("]}\n");
    }

    free(frame);
    free(wavelengths);
    free(spectrum);
    free(continuum);
    free(line_only);
    free(results);
    return 0;
}