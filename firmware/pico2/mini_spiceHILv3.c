/*
 * Projeto: picoHIL - Firmware de simulação de circuitos
 * Autor: Luiz Daniel S. Bezerra
 * Data: Dezembro/2025 - Versão beta inicial.
 *
 * * Licença:
 * Copyright (c) 2025 Luiz Daniel S. Bezerra
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * O SOFTWARE É FORNECIDO "NO ESTADO EM QUE SE ENCONTRA", SEM GARANTIA DE
 * QUALQUER TIPO, EXPRESSA OU IMPLÍCITA, INCLUINDO MAS NÃO SE LIMITANDO ÀS
 * GARANTIAS DE COMERCIALIZAÇÃO, ADEQUAÇÃO A UM PROPÓSITO ESPECÍFICO E
 * NÃO VIOLAÇÃO. EM NENHUM CASO OS AUTORES OU DETENTORES DE DIREITOS SERÃO
 * RESPONSÁVEIS POR QUALQUER RECLAMAÇÃO, DANOS OU OUTRAS RESPONSABILIDADES,
 * SEJA EM AÇÃO DE CONTRATO, ATO ILÍCITO OU OUTRA FORMA, DECORRENTE DE,
 * OU RELACIONADA AO SOFTWARE OU AO USO OU OUTRAS INTERAÇÕES COM O SOFTWARE.
 * 
 */

#include "mini_spiceHILv3.h"

// ======================================================
// UTILITÁRIOS INTERNOS
// ======================================================

static inline float ms_fabs(float x) {
    return x >= 0.0f ? x : -x;
}

// Avalia valor de fonte independente no tempo t
static float ms_source_eval(const ms_source_t *s, float t)
{
    switch (s->type) {
    case MS_SRC_DC:
        return s->dc;

    case MS_SRC_SINE: {
        float omega = 2.0f * (float)M_PI * s->frequency;
        return s->offset + s->amplitude * sinf(omega * t + s->phase);
    }

    case MS_SRC_PULSE: {
        if (t < s->delay)
            return s->v1;

        float td = t - s->delay;
        if (s->period <= 0.0f)
            return s->v1;

        float tt = fmodf(td, s->period);

        if (tt < s->tr) {
            float k = (s->tr > 0.0f) ? (tt / s->tr) : 1.0f;
            return s->v1 + (s->v2 - s->v1) * k;
        }

        if (tt < s->tr + s->width) {
            return s->v2;
        }

        if (tt < s->tr + s->width + s->tf) {
            float k = (s->tf > 0.0f) ? ((tt - (s->tr + s->width)) / s->tf) : 1.0f;
            if (k > 1.0f) k = 1.0f;
            return s->v2 + (s->v1 - s->v2) * k;
        }

        return s->v1;
    }

    case MS_SRC_EXTERNAL:
        if (s->ext == NULL)
            return 0.0f;
        return (*s->ext) * s->gain + s->offset_ext;

    default:
        return 0.0f;
    }
}

// ======================================================
// SOLVERS LINEARES
// ======================================================

// Gauss direto
int ms_gauss_solve(int n,
                          float A[MS_MAX_SIZE][MS_MAX_SIZE],
                          float b[MS_MAX_SIZE],
                          float x[MS_MAX_SIZE])
{
    int i, j, k;

    for (i = 0; i < n; i++) {
        float pivot = A[i][i];
        if (ms_fabs(pivot) < MS_EPSILON) {
            return -1;
        }

        float inv_pivot = 1.0f / pivot;
        for (j = i; j < n; j++) {
            A[i][j] *= inv_pivot;
        }
        b[i] *= inv_pivot;

        for (k = i + 1; k < n; k++) {
            float factor = A[k][i];
            if (ms_fabs(factor) < MS_EPSILON) continue;
            for (j = i; j < n; j++) {
                A[k][j] -= factor * A[i][j];
            }
            b[k] -= factor * b[i];
        }
    }

    for (i = n - 1; i >= 0; i--) {
        float sum = b[i];
        for (j = i + 1; j < n; j++) {
            sum -= A[i][j] * x[j];
        }
        x[i] = sum;
    }

    return 0;
}

// Gauss-Seidel iterativo
int ms_gauss_seidel(int n,
                           float A[MS_MAX_SIZE][MS_MAX_SIZE],
                           float b[MS_MAX_SIZE],
                           float x[MS_MAX_SIZE],
                           int max_iter,
                           float tol)
{
    for (int iter = 0; iter < max_iter; iter++) {
        float max_err = 0.0f;

        for (int i = 0; i < n; i++) {
            float sigma = 0.0f;

            for (int j = 0; j < n; j++) {
                if (j != i)
                    sigma += A[i][j] * x[j];
            }

            float x_new = (b[i] - sigma) / A[i][i];
            float err = ms_fabs(x_new - x[i]);
            if (err > max_err) max_err = err;

            x[i] = x_new;
        }

        if (max_err < tol)
            return 0;
    }

    return 1; // não convergiu
}

// LU decomposition (Doolittle)
int ms_lu_decompose(int n,
                           float A[MS_MAX_SIZE][MS_MAX_SIZE],
                           float L[MS_MAX_SIZE][MS_MAX_SIZE],
                           float U[MS_MAX_SIZE][MS_MAX_SIZE])
{
    for (int i = 0; i < n; i++) {
        // U
        for (int k = i; k < n; k++) {
            float sum = 0.0f;
            for (int j = 0; j < i; j++)
                sum += L[i][j] * U[j][k];
            U[i][k] = A[i][k] - sum;
        }

        // L
        for (int k = i; k < n; k++) {
            if (i == k) {
                L[i][i] = 1.0f;
            } else {
                float sum = 0.0f;
                for (int j = 0; j < i; j++)
                    sum += L[k][j] * U[j][i];
                if (ms_fabs(U[i][i]) < MS_EPSILON)
                    return -1;
                L[k][i] = (A[k][i] - sum) / U[i][i];
            }
        }
    }

    return 0;
}

void ms_lu_solve(int n,
                        float L[MS_MAX_SIZE][MS_MAX_SIZE],
                        float U[MS_MAX_SIZE][MS_MAX_SIZE],
                        float b[MS_MAX_SIZE],
                        float x[MS_MAX_SIZE])
{
    float y[MS_MAX_SIZE];

    for (int i = 0; i < n; i++) {
        float sum = 0.0f;
        for (int j = 0; j < i; j++)
            sum += L[i][j] * y[j];
        y[i] = b[i] - sum;
    }

    for (int i = n - 1; i >= 0; i--) {
        float sum = 0.0f;
        for (int j = i + 1; j < n; j++)
            sum += U[i][j] * x[j];
        x[i] = (y[i] - sum) / U[i][i];
    }
}

// ======================================================
// INICIALIZAÇÃO E ELEMENTOS
// ======================================================

void ms_circuit_init(ms_circuit_t *c, int nodes, float dt)
{
    if (nodes < 1) nodes = 1;
    if (nodes > MS_MAX_NODES) nodes = MS_MAX_NODES;

    c->nodes = nodes;
    c->elems = 0;
    c->t     = 0.0f;
    c->dt    = dt;

    c->system_size = nodes;
    c->solver      = MS_SOLVER_GAUSS;

    for (int i = 0; i < MS_MAX_SIZE; i++) {
        c->b[i] = 0.0f;
        c->x[i] = 0.0f;
        for (int j = 0; j < MS_MAX_SIZE; j++) {
            c->A[i][j] = 0.0f;
        }
    }
}

void ms_set_solver(ms_circuit_t *c, ms_solver_type_t solver)
{
    c->solver = solver;
}

static int ms_add_element_base(ms_circuit_t *c,
                               ms_element_type_t type,
                               int a, int b, float value)
{
    if (c->elems >= MS_MAX_ELEMS)
        return -1;

    ms_element_t *e = &c->elem[c->elems];

    e->type = type;
    e->a    = a;
    e->b    = b;
    e->value = value;

    e->state     = 0.0f;
    e->uses_aux  = 0;
    e->aux_index = -1;

    e->src.type       = MS_SRC_DC;
    e->src.dc         = 0.0f;
    e->src.offset     = 0.0f;
    e->src.amplitude  = 0.0f;
    e->src.frequency  = 0.0f;
    e->src.phase      = 0.0f;
    e->src.v1         = 0.0f;
    e->src.v2         = 0.0f;
    e->src.delay      = 0.0f;
    e->src.tr         = 0.0f;
    e->src.tf         = 0.0f;
    e->src.width      = 0.0f;
    e->src.period     = 0.0f;
    e->src.gain       = 1.0f;
    e->src.offset_ext = 0.0f;
    e->src.ext        = NULL;

    e->c1        = 0;
    e->c2        = 0;
    e->ctrl_elem = -1;
    e->gain      = 0.0f;
    e->ron       = 0.0f;
    e->roff      = 0.0f;
    e->vth       = 0.0f;

    return c->elems++;
}

int ms_add_resistor(ms_circuit_t *c, int a, int b, float R)
{
    return ms_add_element_base(c, MS_ELEM_R, a, b, R);
}

int ms_add_capacitor(ms_circuit_t *c, int a, int b, float C)
{
    return ms_add_element_base(c, MS_ELEM_C, a, b, C);
}

int ms_add_inductor(ms_circuit_t *c, int a, int b, float L)
{
    return ms_add_element_base(c, MS_ELEM_L, a, b, L);
}

ms_rl_series_t ms_add_series_rl_helper(ms_circuit_t *c, int node_a, int node_b, float R, float L)
{
    ms_rl_series_t result = { -1, -1, -1 };

    // Cria nó intermediário automaticamente
    int n_mid = ++c->nodes;

    // Adiciona resistor entre node_a e n_mid
    result.resistor_index = ms_add_resistor(c, node_a, n_mid, R);

    // Adiciona indutor entre n_mid e node_b
    result.inductor_index = ms_add_inductor(c, n_mid, node_b, L);

    result.intermediate_node = n_mid;
    return result;
}

int ms_add_current_source(ms_circuit_t *c, int a, int b, float dc_value)
{
    int idx = ms_add_element_base(c, MS_ELEM_I, a, b, 1.0f);
    if (idx >= 0) {
        c->elem[idx].src.type = MS_SRC_DC;
        c->elem[idx].src.dc   = dc_value;
    }
    return idx;
}

int ms_add_voltage_source(ms_circuit_t *c, int a, int b, float dc_value)
{
    int idx = ms_add_element_base(c, MS_ELEM_V, a, b, 1.0f);
    if (idx >= 0) {
        c->elem[idx].src.type = MS_SRC_DC;
        c->elem[idx].src.dc   = dc_value;
    }
    return idx;
}

int ms_add_sine_source(ms_circuit_t *c, int a, int b,
                           float amplitude, float offset, float freq, float phase_rad) {
    int idx = ms_add_voltage_source(c, a, b, 0.0f); // cria fonte DC=0
    ms_set_source_sine(c, idx, offset, amplitude, freq, phase_rad);
    return idx;
}

// Fontes controladas

int ms_add_vccs(ms_circuit_t *c, int a, int b, int c1, int c2, float gain)
{
    int idx = ms_add_element_base(c, MS_ELEM_VCCS, a, b, gain);
    if (idx >= 0) {
        c->elem[idx].c1   = c1;
        c->elem[idx].c2   = c2;
        c->elem[idx].gain = gain;
    }
    return idx;
}

int ms_add_vcvs(ms_circuit_t *c, int a, int b, int c1, int c2, float gain)
{
    int idx = ms_add_element_base(c, MS_ELEM_VCVS, a, b, gain);
    if (idx >= 0) {
        c->elem[idx].c1   = c1;
        c->elem[idx].c2   = c2;
        c->elem[idx].gain = gain;
    }
    return idx;
}

int ms_add_cccs(ms_circuit_t *c, int a, int b, int ctrl_elem, float gain)
{
    int idx = ms_add_element_base(c, MS_ELEM_CCCS, a, b, gain);
    if (idx >= 0) {
        c->elem[idx].ctrl_elem = ctrl_elem;
        c->elem[idx].gain      = gain;
    }
    return idx;
}

int ms_add_ccvs(ms_circuit_t *c, int a, int b, int ctrl_elem, float gain)
{
    int idx = ms_add_element_base(c, MS_ELEM_CCVS, a, b, gain);
    if (idx >= 0) {
        c->elem[idx].ctrl_elem = ctrl_elem;
        c->elem[idx].gain      = gain;
    }
    return idx;
}

// Switch controlado por tensão
int ms_add_switch(ms_circuit_t *c, int a, int b, int c1, int c2,
                  float ron, float roff, float vth)
{
    int idx = ms_add_element_base(c, MS_ELEM_SWITCH, a, b, ron);
    if (idx >= 0) {
        c->elem[idx].c1   = c1;
        c->elem[idx].c2   = c2;
        c->elem[idx].ron  = ron;
        c->elem[idx].roff = roff;
        c->elem[idx].vth  = vth;
    }
    return idx;
}

// ======================================================
// CONFIGURAÇÃO DE FONTES
// ======================================================

void ms_set_source_sine(ms_circuit_t *c, int elem_index,
                        float offset, float amplitude,
                        float frequency, float phase)
{
    if (elem_index < 0 || elem_index >= c->elems) return;
    ms_element_t *e = &c->elem[elem_index];

    e->src.type      = MS_SRC_SINE;
    e->src.offset    = offset;
    e->src.amplitude = amplitude;
    e->src.frequency = frequency;
    e->src.phase     = phase;
}

void ms_set_source_pulse(ms_circuit_t *c, int elem_index,
                         float v1, float v2,
                         float delay, float tr, float tf,
                         float width, float period)
{
    if (elem_index < 0 || elem_index >= c->elems) return;
    ms_element_t *e = &c->elem[elem_index];

    e->src.type   = MS_SRC_PULSE;
    e->src.v1     = v1;
    e->src.v2     = v2;
    e->src.delay  = delay;
    e->src.tr     = tr;
    e->src.tf     = tf;
    e->src.width  = width;
    e->src.period = period;
}

void ms_set_source_external(ms_circuit_t *c, int elem_index,
                            volatile float *external_value,
                            float gain, float offset)
{
    if (elem_index < 0 || elem_index >= c->elems) return;
    ms_element_t *e = &c->elem[elem_index];

    e->src.type       = MS_SRC_EXTERNAL;
    e->src.ext        = external_value;
    e->src.gain       = gain;
    e->src.offset_ext = offset;
}

// ======================================================
// MONTAGEM DO SISTEMA (MNA)
// ======================================================

static void ms_assemble_system(ms_circuit_t *c)
{
    int N = c->nodes;

    int M = 0;
    for (int i = 0; i < c->elems; i++) {
        ms_element_t *e = &c->elem[i];
        if (e->type == MS_ELEM_V ||
            e->type == MS_ELEM_L ||
            e->type == MS_ELEM_VCVS ||
            e->type == MS_ELEM_CCVS) {
            e->uses_aux  = 1;
            e->aux_index = N + M;
            M++;
        } else {
            e->uses_aux  = 0;
            e->aux_index = -1;
        }
    }

    int size = N + M;
    if (size > MS_MAX_SIZE) size = MS_MAX_SIZE;
    c->system_size = size;

    for (int i = 0; i < size; i++) {
        c->b[i] = 0.0f;
        for (int j = 0; j < size; j++) {
            c->A[i][j] = 0.0f;
        }
    }

    float dt = c->dt;
    float t  = c->t;

    for (int i = 0; i < c->elems; i++) {
        ms_element_t *e = &c->elem[i];

        int a = (e->a == 0 ? -1 : e->a - 1);
        int b = (e->b == 0 ? -1 : e->b - 1);

        switch (e->type) {

        case MS_ELEM_R: {
            float R = e->value;
            if (R <= 0.0f) break;
            float g = 1.0f / R;

            if (a >= 0) c->A[a][a] += g;
            if (b >= 0) c->A[b][b] += g;
            if (a >= 0 && b >= 0) {
                c->A[a][b] -= g;
                c->A[b][a] -= g;
            }
        } break;

        case MS_ELEM_C: {
            float Cval = e->value;
            if (Cval <= 0.0f) break;

            float Gc   = Cval / dt;
            float Vprev= e->state;
            float Ieq  = Gc * Vprev;

            if (a >= 0) {
                c->A[a][a] += Gc;
                c->b[a]    += Ieq;
            }
            if (b >= 0) {
                c->A[b][b] += Gc;
                c->b[b]    -= Ieq;
            }
            if (a >= 0 && b >= 0) {
                c->A[a][b] -= Gc;
                c->A[b][a] -= Gc;
            }
        } break;

        case MS_ELEM_L: {
            float Lval = e->value;
            if (Lval <= 0.0f) break;

            float Req  = Lval / dt;
            float Iprev= e->state;
            float Veq  = - Req * Iprev;

            int k = e->aux_index;
            if (k < 0 || k >= size) break;

            if (a >= 0) {
                c->A[a][k] += 1.0f;
                c->A[k][a] += 1.0f;
            }
            if (b >= 0) {
                c->A[b][k] -= 1.0f;
                c->A[k][b] -= 1.0f;
            }

            c->A[k][k] -= Req;
            c->b[k]    += Veq;
        } break;

        case MS_ELEM_I: {
            float Ival = ms_source_eval(&e->src, t);
            if (a >= 0) c->b[a] -= Ival;
            if (b >= 0) c->b[b] += Ival;
        } break;

        case MS_ELEM_V: {
            float Vval = ms_source_eval(&e->src, t);
            int k = e->aux_index;
            if (k < 0 || k >= size) break;

            if (a >= 0) {
                c->A[a][k] += 1.0f;
                c->A[k][a] += 1.0f;
            }
            if (b >= 0) {
                c->A[b][k] -= 1.0f;
                c->A[k][b] -= 1.0f;
            }

            c->b[k] += Vval;
        } break;

        case MS_ELEM_VCCS: {
            float g = e->gain;
            int c1 = (e->c1 == 0 ? -1 : e->c1 - 1);
            int c2 = (e->c2 == 0 ? -1 : e->c2 - 1);

            if (a >= 0 && c1 >= 0) c->A[a][c1] += g;
            if (a >= 0 && c2 >= 0) c->A[a][c2] -= g;
            if (b >= 0 && c1 >= 0) c->A[b][c1] -= g;
            if (b >= 0 && c2 >= 0) c->A[b][c2] += g;
        } break;

        case MS_ELEM_VCVS: {
            int k = e->aux_index;
            if (k < 0 || k >= size) break;

            int c1 = (e->c1 == 0 ? -1 : e->c1 - 1);
            int c2 = (e->c2 == 0 ? -1 : e->c2 - 1);
            float A_gain = e->gain;

            if (a >= 0) {
                c->A[a][k] += 1.0f;
                c->A[k][a] += 1.0f;
            }
            if (b >= 0) {
                c->A[b][k] -= 1.0f;
                c->A[k][b] -= 1.0f;
            }

            if (c1 >= 0) c->A[k][c1] -= A_gain;
            if (c2 >= 0) c->A[k][c2] += A_gain;
        } break;

        case MS_ELEM_CCCS: {
            int ctrl = e->ctrl_elem;
            if (ctrl < 0 || ctrl >= c->elems) break;
            int kc = c->elem[ctrl].aux_index;
            if (kc < 0 || kc >= size) break;

            float B = e->gain;
            if (a >= 0) c->A[a][kc] += B;
            if (b >= 0) c->A[b][kc] -= B;
        } break;

        case MS_ELEM_CCVS: {
            int k = e->aux_index;
            if (k < 0 || k >= size) break;

            int ctrl = e->ctrl_elem;
            if (ctrl < 0 || ctrl >= c->elems) break;
            int kc = c->elem[ctrl].aux_index;
            if (kc < 0 || kc >= size) break;

            float R = e->gain;

            if (a >= 0) {
                c->A[a][k] += 1.0f;
                c->A[k][a] += 1.0f;
            }
            if (b >= 0) {
                c->A[b][k] -= 1.0f;
                c->A[k][b] -= 1.0f;
            }

            c->A[k][kc] -= R;
        } break;

        case MS_ELEM_SWITCH: {
            int c1 = (e->c1 == 0 ? -1 : e->c1 - 1);
            int c2 = (e->c2 == 0 ? -1 : e->c2 - 1);

            float vctrl = 0.0f;
            if (c1 >= 0) vctrl += c->x[c1];
            if (c2 >= 0) vctrl -= c->x[c2];

            float R = (vctrl > e->vth) ? e->ron : e->roff;
            if (R <= 0.0f) R = e->ron;
            float g = 1.0f / R;

            if (a >= 0) c->A[a][a] += g;
            if (b >= 0) c->A[b][b] += g;
            if (a >= 0 && b >= 0) {
                c->A[a][b] -= g;
                c->A[b][a] -= g;
            }
        } break;

        default:
            break;
        }
    }
}


// Monta apenas elementos fixos (resistores, fontes DC constantes, fontes controladas estáticas)
void ms_assemble_static(ms_circuit_t *c)
{
    int N = c->nodes;
    int M = 0;

    // Define variáveis auxiliares
    for (int i = 0; i < c->elems; i++) {
        ms_element_t *e = &c->elem[i];
        if (e->type == MS_ELEM_V ||
            e->type == MS_ELEM_L ||
            e->type == MS_ELEM_VCVS ||
            e->type == MS_ELEM_CCVS) {
            e->uses_aux  = 1;
            e->aux_index = N + M;
            M++;
        } else {
            e->uses_aux  = 0;
            e->aux_index = -1;
        }
    }

    int size = N + M;
    if (size > MS_MAX_SIZE) size = MS_MAX_SIZE;
    c->system_size = size;

    // Zera matriz e vetor
    for (int i = 0; i < size; i++) {
        c->b[i] = 0.0f;
        for (int j = 0; j < size; j++) {
            c->A[i][j] = 0.0f;
        }
    }

    // Monta elementos estáticos
    for (int i = 0; i < c->elems; i++) {
        ms_element_t *e = &c->elem[i];
        int a = (e->a == 0 ? -1 : e->a - 1);
        int b = (e->b == 0 ? -1 : e->b - 1);

        switch (e->type) {
        case MS_ELEM_R: {
            float R = e->value;
            if (R <= 0.0f) break;
            float g = 1.0f / R;
            if (a >= 0) c->A[a][a] += g;
            if (b >= 0) c->A[b][b] += g;
            if (a >= 0 && b >= 0) {
                c->A[a][b] -= g;
                c->A[b][a] -= g;
            }
        } break;

        case MS_ELEM_VCCS: {
            float g = e->gain;
            int c1 = (e->c1 == 0 ? -1 : e->c1 - 1);
            int c2 = (e->c2 == 0 ? -1 : e->c2 - 1);

            if (a >= 0 && c1 >= 0) c->A[a][c1] += g;
            if (a >= 0 && c2 >= 0) c->A[a][c2] -= g;
            if (b >= 0 && c1 >= 0) c->A[b][c1] -= g;
            if (b >= 0 && c2 >= 0) c->A[b][c2] += g;
        } break;

        case MS_ELEM_VCVS: {
            int k = e->aux_index;
            if (k < 0 || k >= size) break;

            int c1 = (e->c1 == 0 ? -1 : e->c1 - 1);
            int c2 = (e->c2 == 0 ? -1 : e->c2 - 1);
            float A_gain = e->gain;

            if (a >= 0) {
                c->A[a][k] += 1.0f;
                c->A[k][a] += 1.0f;
            }
            if (b >= 0) {
                c->A[b][k] -= 1.0f;
                c->A[k][b] -= 1.0f;
            }

            if (c1 >= 0) c->A[k][c1] -= A_gain;
            if (c2 >= 0) c->A[k][c2] += A_gain;
        } break;

        case MS_ELEM_CCCS: {
            int ctrl = e->ctrl_elem;
            if (ctrl < 0 || ctrl >= c->elems) break;
            int kc = c->elem[ctrl].aux_index;
            if (kc < 0 || kc >= size) break;

            float B = e->gain;
            if (a >= 0) c->A[a][kc] += B;
            if (b >= 0) c->A[b][kc] -= B;
        } break;

        case MS_ELEM_CCVS: {
            int k = e->aux_index;
            if (k < 0 || k >= size) break;

            int ctrl = e->ctrl_elem;
            if (ctrl < 0 || ctrl >= c->elems) break;
            int kc = c->elem[ctrl].aux_index;
            if (kc < 0 || kc >= size) break;

            float R = e->gain;

            if (a >= 0) {
                c->A[a][k] += 1.0f;
                c->A[k][a] += 1.0f;
            }
            if (b >= 0) {
                c->A[b][k] -= 1.0f;
                c->A[k][b] -= 1.0f;
            }

            c->A[k][kc] -= R;
        } break;

        default:
            break;
        }
    }
}

// Atualiza elementos dependentes do tempo/estado
void ms_assemble_dynamic(ms_circuit_t *c)
{
    int size = c->system_size;
    float dt = c->dt;
    float t  = c->t;

    // Zera vetor b (parte dinâmica)
    for (int i = 0; i < size; i++) {
        c->b[i] = 0.0f;
    }

    for (int i = 0; i < c->elems; i++) {
        ms_element_t *e = &c->elem[i];
        int a = (e->a == 0 ? -1 : e->a - 1);
        int b = (e->b == 0 ? -1 : e->b - 1);

        switch (e->type) {
        case MS_ELEM_C: {
            float Cval = e->value;
            if (Cval <= 0.0f) break;
            float Gc   = Cval / dt;
            float Vprev= e->state;
            float Ieq  = Gc * Vprev;
            if (a >= 0) { c->A[a][a] += Gc; c->b[a] += Ieq; }
            if (b >= 0) { c->A[b][b] += Gc; c->b[b] -= Ieq; }
            if (a >= 0 && b >= 0) { c->A[a][b] -= Gc; c->A[b][a] -= Gc; }
        } break;

        case MS_ELEM_L: {
            float Lval = e->value;
            if (Lval <= 0.0f) break;
            float Req  = Lval / dt;
            float Iprev= e->state;
            float Veq  = - Req * Iprev;
            int k = e->aux_index;
            if (k < 0 || k >= size) break;
            c->A[k][k] -= Req;
            c->b[k]    += Veq;
        } break;

        case MS_ELEM_I: {
            float Ival = ms_source_eval(&e->src, t);
            if (a >= 0) c->b[a] -= Ival;
            if (b >= 0) c->b[b] += Ival;
        } break;

        case MS_ELEM_V: {
            float Vval = ms_source_eval(&e->src, t);
            int k = e->aux_index;
            if (k < 0 || k >= size) break;
            c->b[k] += Vval;
        } break;

        case MS_ELEM_SWITCH: {
            int c1 = (e->c1 == 0 ? -1 : e->c1 - 1);
            int c2 = (e->c2 == 0 ? -1 : e->c2 - 1);

            float vctrl = 0.0f;
            if (c1 >= 0) vctrl += c->x[c1];
            if (c2 >= 0) vctrl -= c->x[c2];

            float R = (vctrl > e->vth) ? e->ron : e->roff;
            if (R <= 0.0f) R = e->ron;
            float g = 1.0f / R;

            if (a >= 0) c->A[a][a] += g;
            if (b >= 0) c->A[b][b] += g;
            if (a >= 0 && b >= 0) {
                c->A[a][b] -= g;
                c->A[b][a] -= g;
            }
        } break;

        default:
            break;
        }
    }
}

// Atualiza estados de C e L
static void ms_update_states(ms_circuit_t *c)
{
    for (int i = 0; i < c->elems; i++) {
        ms_element_t *e = &c->elem[i];

        if (e->type == MS_ELEM_C) {
            float Va = 0.0f, Vb = 0.0f;
            if (e->a != 0) Va = c->x[e->a - 1];
            if (e->b != 0) Vb = c->x[e->b - 1];
            e->state = Va - Vb;
        }

        if (e->type == MS_ELEM_L) {
            int k = e->aux_index;
            if (k >= 0 && k < c->system_size) {
                e->state = c->x[k];
            }
        }
    }
}

// ======================================================
// DIAGNÓSTICO DO SISTEMA
// ======================================================
ms_system_status_t ms_check_system(const ms_circuit_t *c)
{
    int n = c->system_size;

    // Verifica elementos inválidos
    for (int i = 0; i < c->elems; i++) {
        const ms_element_t *e = &c->elem[i];
        if ((e->type == MS_ELEM_R && e->value <= 0.0f) ||
            (e->type == MS_ELEM_C && e->value <= 0.0f) ||
            (e->type == MS_ELEM_L && e->value <= 0.0f)) {
            return MS_SYS_INVALID_ELEMENT;
        }
    }

    // Verifica nós isolados (linha toda zero em A)
    for (int i = 0; i < n; i++) {
        int nonzero = 0;
        for (int j = 0; j < n; j++) {
            if (ms_fabs(c->A[i][j]) > MS_EPSILON) {
                nonzero = 1;
                break;
            }
        }
        if (!nonzero) {
            return MS_SYS_ISOLATED_NODE;
        }
    }

    // Verifica pivôs (singularidade simples)
    for (int i = 0; i < n; i++) {
        if (ms_fabs(c->A[i][i]) < MS_EPSILON) {
            return MS_SYS_SINGULAR;
        }
    }

    return MS_SYS_OK;
}

// ======================================================
// PASSO DE SIMULAÇÃO
// ======================================================
int ms_circuit_stepNEW(ms_circuit_t *c)
{
    //
    // Monta parte estática do sistema (resistores, fontes fixas, controladas lineares)
    // ms_assemble_static(&c);
    // Inserir antes do loop principal de execução e apos o
    // a chamada de construcao do circuito.

    // Agora só atualiza parte dinâmica
    ms_assemble_dynamic(c);

    // Checagem de sistema
    ms_system_status_t check = ms_check_system(c);
    if (check != MS_SYS_OK) {
        return check;
    }

    int n = c->system_size;
    int status = 0;

    switch (c->solver) {
    case MS_SOLVER_GAUSS:
        status = ms_gauss_solve(n, c->A, c->b, c->x);
        break;
    case MS_SOLVER_GAUSS_SEIDEL:
        status = ms_gauss_seidel(n, c->A, c->b, c->x, 50, 1e-5f);
        break;
    case MS_SOLVER_LU: {
        static float L[MS_MAX_SIZE][MS_MAX_SIZE];
        static float U[MS_MAX_SIZE][MS_MAX_SIZE];
        status = ms_lu_decompose(n, c->A, L, U);
        if (status == 0) ms_lu_solve(n, L, U, c->b, c->x);
        break;
    }
    }

    if (status != 0) return status;

    ms_update_states(c);
    c->t += c->dt;
    return 0;
}

int ms_circuit_step(ms_circuit_t *c)
{
    ms_assemble_system(c);

    int n = c->system_size;
    int status = 0;

    switch (c->solver) {

    case MS_SOLVER_GAUSS:
        status = ms_gauss_solve(n, c->A, c->b, c->x);
        break;

    case MS_SOLVER_GAUSS_SEIDEL:
        status = ms_gauss_seidel(n, c->A, c->b, c->x,
                                 50,
                                 1e-5f);
        break;

    case MS_SOLVER_LU: {
        static float L[MS_MAX_SIZE][MS_MAX_SIZE];
        static float U[MS_MAX_SIZE][MS_MAX_SIZE];

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                L[i][j] = 0.0f;
                U[i][j] = 0.0f;
            }
        }

        status = ms_lu_decompose(n, c->A, L, U);
        if (status == 0)
            ms_lu_solve(n, L, U, c->b, c->x);
        break;
    }
    }

    if (status != 0)
        return status;

    ms_update_states(c);
    c->t += c->dt;

    // Checagem de sistema
    ms_system_status_t check = ms_check_system(c);
    if (check != MS_SYS_OK) {
        return check;
    }
    //return 0;
}

// ======================================================
// LEITURA DE RESULTADOS
// ======================================================

float ms_get_node_voltage(const ms_circuit_t *c, int node)
{
    if (node == 0) return 0.0f;
    if (node < 0 || node > c->nodes) return 0.0f;
    return c->x[node - 1];
}

float ms_get_element_current(const ms_circuit_t *c, int elem_index)
{
    if (elem_index < 0 || elem_index >= c->elems) return 0.0f;
    const ms_element_t *e = &c->elem[elem_index];
    if (!e->uses_aux) return 0.0f;
    int k = e->aux_index;
    if (k < 0 || k >= c->system_size) return 0.0f;
    return c->x[k];
}
// ======================================================
// CORRENTE EM RESISTORES
// ======================================================
// Corrente em resistor: I = (Va - Vb) / R
float ms_get_resistor_current(const ms_circuit_t *c, int elem_index)
{
    if (elem_index < 0 || elem_index >= c->elems) return 0.0f;
    const ms_element_t *e = &c->elem[elem_index];
    if (e->type != MS_ELEM_R) return 0.0f;

    float Va = 0.0f, Vb = 0.0f;
    if (e->a != 0) Va = ms_get_node_voltage(c, e->a);
    if (e->b != 0) Vb = ms_get_node_voltage(c, e->b);

    if (e->value <= 0.0f) return 0.0f;
    return (Va - Vb) / e->value;
}
// Corrente em capacitor: I = C * dV/dt ≈ C * (V - Vprev)/dt
float ms_get_capacitor_current(const ms_circuit_t *c, int elem_index)
{
    if (elem_index < 0 || elem_index >= c->elems) return 0.0f;
    const ms_element_t *e = &c->elem[elem_index];
    if (e->type != MS_ELEM_C) return 0.0f;

    float Va = 0.0f, Vb = 0.0f;
    if (e->a != 0) Va = ms_get_node_voltage(c, e->a);
    if (e->b != 0) Vb = ms_get_node_voltage(c, e->b);

    float V = Va - Vb;
    float dV = V - e->state; // e->state guarda Vprev
    return e->value * (dV / c->dt);
}

// ======================================================
// LISTAGEM DE COMPONENTES
// ======================================================

const char* ms_element_type_str(ms_element_type_t type)
{
    switch (type) {
    case MS_ELEM_R:     return "Resistor";
    case MS_ELEM_C:     return "Capacitor";
    case MS_ELEM_L:     return "Indutor";
    case MS_ELEM_V:     return "Fonte de tensão";
    case MS_ELEM_I:     return "Fonte de corrente";
    case MS_ELEM_VCVS:  return "Fonte VCVS";
    case MS_ELEM_CCVS:  return "Fonte CCVS";
    case MS_ELEM_VCCS:  return "Fonte VCCS";
    case MS_ELEM_CCCS:  return "Fonte CCCS";
    case MS_ELEM_SWITCH:return "Chave";
    default:            return "Desconhecido";
    }
}

void ms_list_elements(const ms_circuit_t *c)
{
    printf("=== Lista de componentes do circuito ===\n");
    for (int i = 0; i < c->elems; i++) {
        const ms_element_t *e = &c->elem[i];
        printf("[%d] %-15s  (nó %d ↔ nó %d)  valor=%.6g\n",
               i,
               ms_element_type_str(e->type),
               e->a,
               e->b,
               e->value);
    }
    printf("========================================\n");
}
// ======================================================
// INTERFACE PWM/DAC
// ======================================================

uint16_t ms_value_to_pwm_duty(float normalized_value, uint16_t pwm_max)
{
    float v = normalized_value;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return (uint16_t)(v * (float)pwm_max);
}

uint16_t ms_signal_to_pwm(float value,
                          float gain, float offset,
                          uint16_t pwm_max)
{
    float v = value * gain + offset;
    return ms_value_to_pwm_duty(v, pwm_max);
}

uint16_t ms_signal_to_dac(float value,
                          float gain, float offset,
                          uint16_t dac_max)
{
    float v = value * gain + offset;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return (uint16_t)(v * (float)dac_max);
}

const char* ms_system_status_str(int status)
{
    switch (status) {
    case MS_SYS_OK:
        return "OK — solução encontrada";
    case MS_SYS_SINGULAR:
        return "Erro: matriz singular (circuito mal definido)";
    case MS_SYS_INVALID_ELEMENT:
        return "Erro: elemento inválido (R, C ou L <= 0)";
    case MS_SYS_ISOLATED_NODE:
        return "Erro: nó isolado detectado";
    case MS_SYS_SOLVER_PIVOT:
        return "Erro: falha no solver (pivô nulo)";
    case MS_SYS_SOLVER_NOCONV:
        return "Erro: método iterativo não convergiu";
    default:
        return "Erro desconhecido";
    }
}