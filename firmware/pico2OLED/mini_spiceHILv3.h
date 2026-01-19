#ifndef MINI_SPICE_H
#define MINI_SPICE_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"

// ======================================================
// CONFIGURAÇÕES DIVERSAS
// ======================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PWM_WRAP   255   // 8 bits
#define PWM_FREQ_8BIT 586260  // ~ 586kHz

// ======================================================
// CONFIGURAÇÕES GERAIS
// ======================================================

#define MS_MAX_NODES   16
#define MS_MAX_ELEMS   64
#define MS_MAX_SIZE   (MS_MAX_NODES + MS_MAX_ELEMS)

#define MS_EPSILON     1e-9f
// ======================================================
// DIAGNÓSTICO DO SISTEMA
// ======================================================

typedef enum {
    MS_SYS_OK = 0,             // Tudo certo
    MS_SYS_SINGULAR = -2,      // Matriz singular
    MS_SYS_INVALID_ELEMENT = -3,// Elemento inválido
    MS_SYS_ISOLATED_NODE = -4, // Nó isolado

    MS_SYS_SOLVER_PIVOT = -1,  // Falha no solver (pivô nulo)
    MS_SYS_SOLVER_NOCONV = 1   // Solver iterativo não convergiu
} ms_system_status_t;

// ======================================================
// TIPOS DE ELEMENTOS
// ======================================================

typedef enum {
    MS_ELEM_R,        // Resistor
    MS_ELEM_C,        // Capacitor
    MS_ELEM_L,        // Indutor
    MS_ELEM_I,        // Fonte de corrente
    MS_ELEM_V,        // Fonte de tensão

    // Fontes controladas
    MS_ELEM_VCVS,     // E — tensão controlada por tensão
    MS_ELEM_VCCS,     // G — corrente controlada por tensão
    MS_ELEM_CCVS,     // H — tensão controlada por corrente
    MS_ELEM_CCCS,     // F — corrente controlada por corrente

    // Interruptor controlado por tensão
    MS_ELEM_SWITCH,
    // Diodo
    MS_ELEM_DIODE
} ms_element_type_t;

// ======================================================
// TIPOS DE FONTES
// ======================================================

typedef enum {
    MS_SRC_DC,
    MS_SRC_SINE,
    MS_SRC_PULSE,
    MS_SRC_EXTERNAL
} ms_source_type_t;

// ======================================================
// SOLVERS
// ======================================================

typedef enum {
    MS_SOLVER_GAUSS,
    MS_SOLVER_GAUSS_SEIDEL,
    MS_SOLVER_LU
} ms_solver_type_t;

// ======================================================
// ESTRUTURA DE FONTE
// ======================================================

typedef struct {
    ms_source_type_t type;

    // DC
    float dc;

    // Senoidal
    float offset;
    float amplitude;
    float frequency;
    float phase;

    // Pulso
    float v1, v2;
    float delay, tr, tf, width, period;

    // EXTERNAL (ADC)
    float gain;
    float offset_ext;
    volatile float *ext;
} ms_source_t;


// ======================================================
// Adiciona malha RL em série entre dois nós
// ======================================================

typedef struct {
    int resistor_index;
    int inductor_index;
    int intermediate_node;
} ms_rl_series_t;

// ======================================================
// ESTRUTURA DE ELEMENTO
// ======================================================

typedef struct {
    ms_element_type_t type;

    int a, b;          // nós principais
    float value;       // R, L, C, etc.

    float state;       // estado de C e L

    int uses_aux;      // se usa variável auxiliar
    int aux_index;     // índice da variável auxiliar

    ms_source_t src;   // fonte associada (para I e V)

    // Fontes controladas
    int c1, c2;        // nós de controle (VCVS, VCCS)
    int ctrl_elem;     // elemento controlado (CCVS, CCCS)
    float gain;        // ganho da fonte controlada

    // Interruptor
    float ron, roff, vth;
    // Diodo
    float vf;
} ms_element_t;

// ======================================================
// ESTRUTURA DO CIRCUITO
// ======================================================

typedef struct {
    int nodes;      // número de nós do circuito
    int elems;      // número de elementos (resistores, fontes, diodos, etc.)
    float t;        // tempo atual da simulação
    float dt;       // passo de tempo

    ms_element_t elem[MS_MAX_ELEMS];        // vetor com todos os elementos do circuito

    float A[MS_MAX_SIZE][MS_MAX_SIZE];      // matriz do sistema (condutâncias + vínculos)
    float b[MS_MAX_SIZE];       // vetor independente (correntes/fontes)
    float x[MS_MAX_SIZE];       // solução (tensões nos nós e correntes auxiliares)

    int system_size;            // tamanho efetivo do sistema linear

    ms_solver_type_t solver;    // tipo de solver usado (ex: Gauss, LU, etc.)
} ms_circuit_t;

// ======================================================
// API PRINCIPAL
// ======================================================

void ms_circuit_init(ms_circuit_t *c, int nodes, float dt);
void ms_set_solver(ms_circuit_t *c, ms_solver_type_t solver);

// Elementos básicos
int ms_add_resistor   (ms_circuit_t *c, int a, int b, float R);
int ms_add_capacitor  (ms_circuit_t *c, int a, int b, float C);
int ms_add_inductor   (ms_circuit_t *c, int a, int b, float L);
ms_rl_series_t ms_add_series_rl_helper(ms_circuit_t *c, int node_a, int node_b, float R, float L);
int ms_add_current_source(ms_circuit_t *c, int a, int b, float dc_value);
int ms_add_voltage_source(ms_circuit_t *c, int a, int b, float dc_value);
int ms_add_sine_source(ms_circuit_t *c, int a, int b, 
            float amplitude, float offset, float freq, float phase_rad);

// Fontes controladas
int ms_add_vccs(ms_circuit_t *c, int a, int b, int c1, int c2, float gain);
int ms_add_vcvs(ms_circuit_t *c, int a, int b, int c1, int c2, float gain);
int ms_add_cccs(ms_circuit_t *c, int a, int b, int ctrl_elem, float gain);
int ms_add_ccvs(ms_circuit_t *c, int a, int b, int ctrl_elem, float gain);

// Interruptor controlado por tensão
int ms_add_switch(ms_circuit_t *c, int a, int b, int c1, int c2,
                  float ron, float roff, float vth);
// Diodo idealizado.                  
int ms_add_diode(ms_circuit_t *c, int anode, int cathode,
                 float ron, float roff, float vf);                  

// Configuração de fontes
void ms_set_source_sine(ms_circuit_t *c, int elem_index,
                        float offset, float amplitude,
                        float frequency, float phase);

void ms_set_source_pulse(ms_circuit_t *c, int elem_index,
                         float v1, float v2,
                         float delay, float tr, float tf,
                         float width, float period);

void ms_set_source_external(ms_circuit_t *c, int elem_index,
                            volatile float *external_value,
                            float gain, float offset);

// Simulação
int   ms_circuit_step(ms_circuit_t *c);

// Conferencia por erros
ms_system_status_t ms_check_system(const ms_circuit_t *c);
const char* ms_system_status_str(int status);

// Leitura
float ms_get_node_voltage(const ms_circuit_t *c, int node);
float ms_get_element_current(const ms_circuit_t *c, int elem_index);
float ms_get_resistor_current(const ms_circuit_t *c, int elem_index);
float ms_get_capacitor_current(const ms_circuit_t *c, int elem_index);
void ms_list_elements(const ms_circuit_t *c);
// ======================================================
// INTERFACE COM PWM/DAC
// ======================================================

uint16_t ms_value_to_pwm_duty(float normalized_value, uint16_t pwm_max);

uint16_t ms_signal_to_pwm(float value,
                          float gain, float offset,
                          uint16_t pwm_max);

uint16_t ms_signal_to_dac(float value,
                          float gain, float offset,
                          uint16_t dac_max);

#endif // MINI_SPICE_H
