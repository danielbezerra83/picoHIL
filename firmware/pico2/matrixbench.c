#include <stdio.h>
#include "mini_spiceHILv3.h"
#include "pico/stdlib.h"

extern uint64_t micros();

extern int ms_gauss_solve(int n,
                          float A[MS_MAX_SIZE][MS_MAX_SIZE],
                          float b[MS_MAX_SIZE],
                          float x[MS_MAX_SIZE]);
extern int ms_gauss_seidel(int n,
                           float A[MS_MAX_SIZE][MS_MAX_SIZE],
                           float b[MS_MAX_SIZE],
                           float x[MS_MAX_SIZE],
                           int max_iter,
                           float tol);
extern int ms_lu_decompose(int n,
                           float A[MS_MAX_SIZE][MS_MAX_SIZE],
                           float L[MS_MAX_SIZE][MS_MAX_SIZE],
                           float U[MS_MAX_SIZE][MS_MAX_SIZE]);
extern void ms_lu_solve(int n,
                        float L[MS_MAX_SIZE][MS_MAX_SIZE],
                        float U[MS_MAX_SIZE][MS_MAX_SIZE],
                        float b[MS_MAX_SIZE],
                        float x[MS_MAX_SIZE]);
// ======================================================
// BENCHMARK DE MATRIZES
// ======================================================
void benchmark_matrices() {
    const int sizes[] = {3, 5, 10};
    for (int s = 0; s < 3; s++) {
        int n = sizes[s];
        float A[MS_MAX_SIZE][MS_MAX_SIZE];
        float b[MS_MAX_SIZE];
        float x[MS_MAX_SIZE];

        // Preenche matriz com valores sofisticados
        for (int i = 0; i < n; i++) {
            b[i] = sinf(i + 1);
            for (int j = 0; j < n; j++) {
                A[i][j] = cosf(i + j + 1) + (i == j ? n : 0.5f);
            }
            x[i] = 0.0f;
        }

        uint64_t t0 = micros();
        ms_gauss_solve(n, A, b, x);
        uint64_t t1 = micros();
        printf("Gauss %dx%d: %llu us\n", n, n, (t1 - t0));

        // Recarrega A/b para novo teste
        for (int i = 0; i < n; i++) {
            b[i] = sinf(i + 1);
            for (int j = 0; j < n; j++) {
                A[i][j] = cosf(i + j + 1) + (i == j ? n : 0.5f);
            }
            x[i] = 0.0f;
        }

        t0 = micros();
        ms_gauss_seidel(n, A, b, x, 100, 1e-5f);
        t1 = micros();
        printf("Gauss-Seidel %dx%d: %llu us\n", n, n, (t1 - t0));

        // LU
        float L[MS_MAX_SIZE][MS_MAX_SIZE] = {0};
        float U[MS_MAX_SIZE][MS_MAX_SIZE] = {0};
        for (int i = 0; i < n; i++) {
            b[i] = sinf(i + 1);
            for (int j = 0; j < n; j++) {
                A[i][j] = cosf(i + j + 1) + (i == j ? n : 0.5f);
            }
            x[i] = 0.0f;
        }

        t0 = micros();
        ms_lu_decompose(n, A, L, U);
        ms_lu_solve(n, L, U, b, x);
        t1 = micros();
        printf("LU %dx%d: %llu us\n", n, n, (t1 - t0));
    }
}