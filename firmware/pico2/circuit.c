/*
 * Projeto: picoHIL - Firmware de simulação de circuitos
 * Autor: Luiz Daniel S. Bezerra
 * Data: Dezembro/2025 - Versão beta inicial.
 *
 * Descrição:
 * Este firmware implementa rotinas de simulação de circuitos elétricos
 * (R, L, C, fontes e elementos controlados) para uso em sistemas embarcados,
 * como o Raspberry Pi Pico2.
 * 
 * Emula com excelente precisao Circuitos Simples;
 * No Raspberry Pi Pico 2 estão disponíveis as entradas ADC0[26], ADC1[27], ADC2[28]
 * que podem iteragir com o circuito sendo fontes de tensão ou corrente.
 * A saída do circuito é realizada através de 4 PWM+LPF (2.2k com 10nF), que podem
 * ler as tensões nos nós ou a corrente através dos elementos.
 * O código foi feito em C para máxima optmizacao dos recuros limitados e aproveitamento
 * da CPU do Pico2.
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
#include "hardware/pwm.h"

// ======================================================
// DEFINIÇÕES PARA SELECIONAR O EXEMPLO DE CIRCUITO
// SELECIONE APENAS UM.
// ======================================================
#define EXEMPLO_RLC_SIMPLE_V1       1 // <== ATIVO
#define EXEMPLO_RLC_SIMPLE_V2       0
#define EXEMPLO_RL_SIMPLE           0
#define EXEMPLO_RL_MULTISOURCE      0
#define EXEMPLO_TRIFASICO_V1        0
#define EXEMPLO_TRIFASICO_V2        0
#define MY_CIRCUIT        0


// Prototipos de funcoes.
void setup_rl_simple(ms_circuit_t *c, volatile float *adc_in);
void setup_rlc_circuit_simple(ms_circuit_t *c, volatile float *adc_in);
void setup_rlc_circuit_simpleV2(ms_circuit_t *c, volatile float *adc_in);
void setup_rl_harmonics(ms_circuit_t *c, volatile float *adc_in);
void setup_rl_multiplesource(ms_circuit_t *c, volatile float *adc_in);

void outputs_three_phase_rl(ms_circuit_t *c);
void setup_three_phase_rl(ms_circuit_t *c, volatile float *adc_in);
void setup_three_phase_rl2(ms_circuit_t *c, volatile float *adc_in);
void update_3f_sources(ms_circuit_t *c, volatile float *adc_in);

void setup_my_circuit(ms_circuit_t *c, volatile float *adc_in);
void custom_update_sources(ms_circuit_t *c, volatile float *adc_in);

// Declaracao principal do circuito
void setup_circuit(ms_circuit_t *c, volatile float *adc_in)
{
#if EXEMPLO_RLC_SIMPLE_V1
    setup_rlc_circuit_simple(c, adc_in);
#elif EXEMPLO_RLC_SIMPLE_V2
    setup_rlc_circuit_simpleV2(c, adc_in);
#elif EXEMPLO_RL_SIMPLE
    setup_rl_simple(c, adc_in);
#elif EXEMPLO_RL_MULTISOURCE
    setup_rl_multiplesource(c, adc_in);
#elif EXEMPLO_TRIFASICO_V1
    setup_three_phase_rl(c, adc_in);
#elif EXEMPLO_TRIFASICO_V2
    setup_three_phase_rl2(c, adc_in);
#endif
} 

void output_circuit(ms_circuit_t *c)
{
    // Para visualizar a saida do circuito nos PWM+LPF (PWMDAC simples)
    // iL = ms_get_element_current(c, 3); // como exemplo a corrente no indutor de indice 3
    // os indices podem ser consultados no inicio da execucao 
    // === Lista de componentes do circuito ===
    // [0] Fonte de tensão  (nó 1 ↔ nó 3)  valor=1.00000
    // [1] Fonte de tensão  (nó 3 ↔ nó 0)  valor=1.00000
    // [2] Resistor         (nó 1 ↔ nó 2)  valor=1.00000
    // [3] Indutor          (nó 2 ↔ nó 0)  valor=0.0100000
    // [4] Resistor         (nó 2 ↔ nó 0)  valor=50000.0
    // [5] Resistor         (nó 3 ↔ nó 0)  valor=10.00000e+05
    // ========================================
    // se emprega a funcao ms_signal_to_pwm(iL, 1.0f, 0.5f, PWM_WRAP);
    // para converter o valor simulado (exemplo: corrente)
    // . o primeiro parametro diz respeito a variavel a ser medida (iL)
    // . o segundo é o ganho => nesse caso deve ser ajustado para a escala apropriada
    //      se quiser normalizar em torno de 3.3V 
    //      (0V => 3.3V na simulacao será 0V=>3.3V na saida do PWMDAC),
    //      basta por o ganho c/ 1.0f/3.3f
    // . o terceiro eh o offset => 0.5f coloca a saída no meio do 3.3V do PWMDAC (1.65V)
    // para sinais que apresentam bipolaridade (-/+) é importante manter com esse offset.
    // caso queira visualizar apenas a parte positiva do sinal, colocar esse valor em 0.0f
    // . o quarto eh sempre PWM_WRAP
    //==============================================

#if EXEMPLO_TRIFASICO_V1 || EXEMPLO_TRIFASICO_V2
    outputs_three_phase_rl(c);
#elif  EXEMPLO_RLC_SIMPLE_V1 
    float iL, vC, v1; 
    // Corrente no indutor (elemento 1)
    iL = ms_get_element_current(c, 1);
    // Atualiza PWM com corrente normalizada
    uint16_t duty = ms_signal_to_pwm(iL, 1.0f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(16), PWM_CHAN_A, duty);

    vC = ms_get_node_voltage(c, 2); // Tensao no capacitor
    duty = ms_signal_to_pwm(vC, 1.0f/3.3f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(17), PWM_CHAN_B, duty); 

    // Tensao no RLC para visualizar o que eh aplicado ao circuito
    v1 = ms_get_node_voltage(c, 1); 
    duty = ms_signal_to_pwm(v1, 1.0f/3.3f, 0.5f, PWM_WRAP);     
    pwm_set_chan_level(pwm_gpio_to_slice_num(18), PWM_CHAN_A, duty); 
#elif EXEMPLO_RLC_SIMPLE_V2
    float iR, vC;   
    // Corrente no resistor-serie com indutor (elemento 2)
    iR = ms_get_resistor_current(c, 2);
    // Atualiza PWM com corrente normalizada
    uint16_t duty = ms_signal_to_pwm(iR, 1.0f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(16), PWM_CHAN_A, duty);

    vC = ms_get_node_voltage(c, 3); // Tensao no capacitor
    duty = ms_signal_to_pwm(vC, 1.0f/3.3f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(17), PWM_CHAN_B, duty); 
#elif EXEMPLO_RL_SIMPLE 
    /***********************************************/
    // Para o exemplo setup_rl_simple()
    float V1, iR1, vL1;

    V1 = ms_get_node_voltage(c, 1);
    // Normaliza 0–3.3 V para 0–1
    uint16_t duty = ms_signal_to_pwm(V1, 1.0f/3.3f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(16), PWM_CHAN_A, duty);
    
    //iL1 = ms_get_element_current(c, 2);   // Medicao de corrente no indutor
    iR1 = ms_get_resistor_current(c, 1);    // Medicao de corrente no resistor
    duty = ms_signal_to_pwm(iR1, 1.00f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(17), PWM_CHAN_B, duty);  

    vL1 = ms_get_node_voltage(c, 2);    // Tensao no indutor.
    duty = ms_signal_to_pwm(vL1, 1.0f/3.3f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(18), PWM_CHAN_A, duty);  
     /**********************************************/
#elif EXEMPLO_RL_MULTISOURCE
    /***********************************************/
    // Para o exemplo setup_rl_simple()
    float V1, iR1, vL1, iL1;

    V1 = ms_get_node_voltage(c, 1);
    // Normaliza 0–3.3 V para 0–1
    uint16_t duty = ms_signal_to_pwm(V1, 1.0f/3.3f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(16), PWM_CHAN_A, duty);
    
    iR1 = ms_get_resistor_current(c, 2);    // Corrente atraves do resistor.
    duty = ms_signal_to_pwm(iR1, 1.00f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(17), PWM_CHAN_B, duty);  
    
    vL1 = ms_get_node_voltage(c, 2); // Tensao no indutor
    duty = ms_signal_to_pwm(vL1, 1.0f/3.3f, 0.5f, PWM_WRAP);
    //iL1 = ms_get_element_current(c, 3); // Corrente através do indutor
    //duty = ms_signal_to_pwm(iL1, 1.00f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(18), PWM_CHAN_A, duty);  
    /**********************************************/
#else
    // my_circuit

#endif
}
// ======================================================
// CIRCUITO CUSTOMIZADO
// ======================================================

void setup_my_circuit(ms_circuit_t *c, volatile float *adc_in)
{
    // Insira aqui o código do circuito que voce quer descrever

}

void custom_update_sources(ms_circuit_t *c, volatile float *adc_in)
{
    // Insira aqui o código de atualizacao em tempo real
    // dos valores que definem as fontes 
    // (amplitude, fase, frequencia, por exemplo)

}

//*******************************************************/
//*******************************************************/
//*******************************************************/
//*******************************************************/
void update_sources(ms_circuit_t *c, volatile float *adc_in)
{
#if EXEMPLO_TRIFASICO_V2
    update_3f_sources(c, adc_in);
#else
    custom_update_sources(c, adc_in);
#endif
}
// ======================================================
// CIRCUITO RLC SIMPLES
// ======================================================
void setup_rlc_circuit_simple(ms_circuit_t *c, volatile float *adc_in) {
    ms_circuit_init(c, 2, 100e-6f); // 2 nós, dt=100us

    // R entre nó1 e terra
    ms_add_resistor(c, 1, 0, 1000.0f); // 1kΩ

    // L entre nó1 e nó2
    ms_add_inductor(c, 1, 2, 10e-3f); // 10mH

    // C entre nó2 e terra
    ms_add_capacitor(c, 2, 0, 33e-6f); // 33µF

    // Fonte de tensão controlada pelo ADC
    int Vsrc = ms_add_voltage_source(c, 1, 0, 0.0f);
    // Essa parametrizacao da fonte remove o +1.65V que esta presente no ADC, tornando
    // a fonte de tensao conectada ao circuito bipolar.
    ms_set_source_external(c, Vsrc, adc_in, 3.3f, -1.650f); // ADC normalizado * 3.3V + offset
}

// ======================================================
// CIRCUITO RLC SIMPLES v2
// ======================================================
void setup_rlc_circuit_simpleV2(ms_circuit_t *c, volatile float *adc_in) {
    ms_circuit_init(c, 3, 100e-6f); // 3 nós, dt=100us
    // R entre nó1 e terra
    ms_add_resistor(c, 1, 0, 1000.0f); // 1kΩ

    // Lseries
    // L entre nó1 e nó2
    ms_add_inductor(c, 1, 2, 10e-3f); // 10mH
    // Rser entre nó2 e no3
    ms_add_resistor(c, 2, 3, 5.0f);   // 5.0Ω
    // C entre nó3 e terra
    ms_add_capacitor(c, 3, 0, 33e-6f); // 33µF

    // Fonte de tensão controlada pelo ADC
    int Vsrc = ms_add_voltage_source(c, 1, 0, 0.0f);
    // Essa parametrizacao da fonte remove o +1.65V que esta presente no ADC, tornando
    // a fonte de tensao conectada ao circuito bipolar.
    ms_set_source_external(c, Vsrc, adc_in, 3.3f, -1.650f); // ADC normalizado * 3.3V + offset
}

// ======================================================
// CIRCUITO COM UNICA FONTE EXTERNA
// ======================================================
void setup_rl_simple(ms_circuit_t *c, volatile float *adc_in) {
    // Inicializa circuito com 2 nós (mais terra = nó 0)
    ms_circuit_init(c, 2, 100e-6f); // dt = 100 µs

    // Fonte de tensão controlada pelo ADC (sem resistor paralelo!)
    int Vsrc = ms_add_voltage_source(c, 1, 0, 0.0f);
    ms_set_source_external(c, Vsrc, adc_in, 3.3f, -1.650f); 
    // adc_in normalizado (0–1) → 0–3.3 V

    // Circuito RL em série:
    // Nó1 -> R -> Nó2 -> L -> Terra
    ms_add_resistor(c, 1, 2, 1.00f);    // R = 1.0 Ω
    ms_add_inductor(c, 2, 0, 0.01f);     // L = 10.0 mH

    // Resistor de carga para terra (evita nó flutuante)
    ms_add_resistor(c, 2, 0, 50.0e3f);   // R = 50 kΩ
}


// ======================================================
// CIRCUITO COMBINADO DE FONTE + EXTERNA
// ======================================================
void setup_rl_multiplesource(ms_circuit_t *c, volatile float *adc_in) {
    // Inicializa circuito com 3 nós (mais terra = nó 0)
    ms_circuit_init(c, 3, 100e-6f); // dt = 100 µs

    // Fonte de tensão controlada pelo ADC entre nó1 e nó3
    int Vadc = ms_add_voltage_source(c, 1, 3, 0.0f);
    ms_set_source_external(c, Vadc, adc_in, 3.3f, -1.650f);

    // Fonte senoidal entre nó3 e terra
    int Vsine = ms_add_voltage_source(c, 3, 0, 0.0f);
    ms_set_source_sine(c, Vsine, 0.0f, 0.25f, 60.0f, 0.0f);
    // offset=0, amplitude=0.25 V, freq=60 Hz, fase=0

    // Circuito RL em série:
    // Nó1 -> R -> Nó2 -> L -> Terra
    ms_add_resistor(c, 1, 2, 1.00f);    // R = 1.0 Ω
    ms_add_inductor(c, 2, 0, 0.01f);     // L = 10.0 mH

    // Resistor de carga para terra (evita nó flutuante)
    ms_add_resistor(c, 2, 0, 50.0e3f);   // R = 50 kΩ
    // resistor de estabilização
    ms_add_resistor(c, 3, 0, 1.0e6f); // resistor de estabilização
}


// ======================================================
// CIRCUITO COMBINADO TRIFASICO RL + 3 FONTES + EXTERNA
// ======================================================
void setup_three_phase_rl(ms_circuit_t *c, volatile float *adc_in)
{
    // Nós: 3 fases (A=1, B=2, C=3) + fonte externa (D=4) + terra (0)
    // 
    ms_circuit_init(c, 7, 150e-6f); // dt = 100 µs

    // Parâmetros da fonte
    const float V_amp = 1.4242f;   // ~1.0 Vrms
    const float freq  = 60.0f;    // 60 Hz

    // Fontes senoidais por fase (tensão fase-terra)
    int Va = ms_add_voltage_source(c, 1, 4, 0.0f);
    int Vb = ms_add_voltage_source(c, 2, 4, 0.0f);
    int Vc = ms_add_voltage_source(c, 3, 4, 0.0f);

    // Fonte de tensão controlada pelo ADC entre nó4 e nó0
    int Vadc = ms_add_voltage_source(c, 4, 0, 0.0f);
    ms_set_source_external(c, Vadc, adc_in, 3.3f, -1.650f);

    // Offset=0, amplitude=V_amp, freq=freq, fase em graus
    ms_set_source_sine(c, Va, 0.0f, V_amp, freq,   0.0f);    // Fase A
    ms_set_source_sine(c, Vb, 0.0f, V_amp, freq, 120.0f/(180.0f/M_PI));    // Fase B (+120°)
    ms_set_source_sine(c, Vc, 0.0f, V_amp, freq, 240.0f/(180.0f/M_PI));    // Fase C (+240°)

    // Carga RL por fase (estrela para terra)
    const float R = 10.0f;
    const float L = 50e-3f;

    // Fase A: nó 1 -> R -> nó 5 -> L -> terra
    int Ra = ms_add_resistor(c, 1, 5, R);
    int La = ms_add_inductor(c, 5, 0, L);

    // Fase B: nó 2 -> R -> nó 6 -> L -> terra
    int Rb = ms_add_resistor(c, 2, 6, R);
    int Lb = ms_add_inductor(c, 6, 0, L);

    // Fase C: nó 3 -> R -> nó 7 -> L -> terra
    int Rc = ms_add_resistor(c, 3, 7, R);
    int Lc = ms_add_inductor(c, 7, 0, L);

    // Resistores para resolver o problema de pivo nulo
    ms_add_resistor(c, 1, 4, R*100.0f);
    ms_add_resistor(c, 2, 4, R*100.0f);
    ms_add_resistor(c, 3, 4, R*100.0f);
    int Rcc = ms_add_resistor(c, 4, 0, R*100.0f);
}

// ======================================================
// CIRCUITO COMBINADO TRIFASICO RL + 3 FONTES + EXTERNA
// ======================================================
volatile int Va, Vb, Vc; // Para os indices das fontes
// Parâmetros da fonte
const float V_amp = 1.4242f;   // ~1.0 Vrms
const float freq  = 60.0f;    // 60 Hz
void setup_three_phase_rl2(ms_circuit_t *c, volatile float *adc_in)
{
    // Nós: 3 fases (A=1, B=2, C=3) + fonte externa (D=4) + terra (0)
    // 
    ms_circuit_init(c, 6, 100e-6f); // dt = 100 µs

    // Fontes senoidais por fase (tensão fase-terra)
    Va = ms_add_voltage_source(c, 1, 0, 0.0f);
    Vb = ms_add_voltage_source(c, 2, 0, 0.0f);
    Vc = ms_add_voltage_source(c, 3, 0, 0.0f);

    // Offset=0, amplitude=V_amp, freq=freq, fase em graus
    ms_set_source_sine(c, Va, 0.0f, V_amp, freq,   0.0f);    // Fase A
    ms_set_source_sine(c, Vb, 0.0f, V_amp, freq, 120.0f/(180.0f/M_PI));    // Fase B (+120°)
    ms_set_source_sine(c, Vc, 0.0f, V_amp, freq, 240.0f/(180.0f/M_PI));    // Fase C (+240°)

    // Carga RL por fase (estrela para terra)
    const float R = 10.0f;
    const float L = 50e-3f;

    // Fase A: nó 1 -> R -> nó 4 -> L -> terra
    int Ra = ms_add_resistor(c, 1, 4, R);
    int La = ms_add_inductor(c, 4, 0, L);

    // Fase B: nó 2 -> R -> nó 5 -> L -> terra
    int Rb = ms_add_resistor(c, 2, 5, R);
    int Lb = ms_add_inductor(c, 5, 0, L);

    // Fase C: nó 3 -> R -> nó 6 -> L -> terra
    int Rc = ms_add_resistor(c, 3, 6, R);
    int Lc = ms_add_inductor(c, 6, 0, L);

    // Resistores para resolver o problema de pivo nulo
    ms_add_resistor(c, 1, 0, R*100.0f);
    ms_add_resistor(c, 2, 0, R*100.0f);
    ms_add_resistor(c, 3, 0, R*100.0f);

}

// ======================================================
// Atualiza amplitude das fontes trifásicas conforme ADC
// ======================================================
void update_3f_sources(ms_circuit_t *c, volatile float *adc_in)
{
    // Normaliza ADC (0..3.3 V → 0..1)
    float gain = (*adc_in);
    float V_amp_eff = V_amp * gain;

    // Atualiza cada fonte senoidal com nova amplitude
    ms_set_source_sine(c, Va, 0.0f, V_amp_eff, freq,   0.0f);   // Fase A
    ms_set_source_sine(c, Vb, 0.0f, V_amp_eff, freq, 120.0f/(180.0f/M_PI));   // Fase B
    ms_set_source_sine(c, Vc, 0.0f, V_amp_eff, freq, 240.0f/(180.0f/M_PI));   // Fase C
}

void outputs_three_phase_rl(ms_circuit_t *c)
{
    float Va = ms_get_node_voltage(c, 1);
    float Vb = ms_get_node_voltage(c, 2);
    float Vc = ms_get_node_voltage(c, 3);

    // Normalização simples 0–1 a partir de ±V_amp
    const float V_max = 1.650f;
    uint16_t dA = ms_signal_to_pwm(Va, 1.0f/(2.0f*V_max), 0.5f, PWM_WRAP + 1);
    uint16_t dB = ms_signal_to_pwm(Vb, 1.0f/(2.0f*V_max), 0.5f, PWM_WRAP + 1);
    uint16_t dC = ms_signal_to_pwm(Vc, 1.0f/(2.0f*V_max), 0.5f, PWM_WRAP + 1);

    pwm_set_chan_level(pwm_gpio_to_slice_num(16), PWM_CHAN_A, dA);
    pwm_set_chan_level(pwm_gpio_to_slice_num(17), PWM_CHAN_B, dB);
    pwm_set_chan_level(pwm_gpio_to_slice_num(18), PWM_CHAN_A, dC);


    float iR1 = ms_get_resistor_current(c, 3);
    uint16_t duty = ms_signal_to_pwm(iR1, 1.00f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(19), PWM_CHAN_B, duty); 

    /***************************************************************
    // Correntes nas fases (guardadas em user_i[])
    float Ia = ms_get_element_current(c, c->user_i[0]);
    float Ib = ms_get_element_current(c, c->user_i[1]);
    float Ic = ms_get_element_current(c, c->user_i[2]);

    // Se quiser mandar correntes para outros PWMs:
    const float I_max = V_max / R; // corrente pico aproximada
    uint16_t diA = ms_signal_to_pwm(Ia, 1.0f/(2.0f*I_max), 0.5f, PWM_WRAP + 1);
    uint16_t diB = ms_signal_to_pwm(Ib, 1.0f/(2.0f*I_max), 0.5f, PWM_WRAP + 1);
    uint16_t diC = ms_signal_to_pwm(Ic, 1.0f/(2.0f*I_max), 0.5f, PWM_WRAP + 1);

    pwm_set_chan_level(pwm_gpio_to_slice_num(19), PWM_CHAN_B, diA);
    ***************************************************************/
}