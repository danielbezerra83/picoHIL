
#include "mini_spiceHILv3.h"
#include "hardware/pwm.h"

// Prototipos de funcoes.
void setup_rlc_circuit_simpleV2(ms_circuit_t *c, volatile float *adc_in);
void setup_rl_harmonics(ms_circuit_t *c, volatile float *adc_in);
void setup_rl_multiplesource(ms_circuit_t *c, volatile float *adc_in);

// Declaracao principal do circuito
void setup_circuit(ms_circuit_t *c, volatile float *adc_in)
{
    //====================================================
    // Remover o comentário e executar apenas um circuito.
    //====================================================
    //setup_rlc_circuit_simpleV2(c, adc_in);
    //setup_rl_harmonics(c, adc_in);
    setup_rl_multiplesource(c, adc_in);
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


    /************************************************
    //Para os exemplos rlc_circuit_simple()
    float iL, vC; 
    // Corrente no indutor (elemento 1)
    iL = ms_get_element_current(c, 1);
    vC = ms_get_node_voltage(c, 3);
    // Atualiza PWM com corrente normalizada
    uint16_t duty = ms_signal_to_pwm(iL, 1.0f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(16), PWM_CHAN_A, duty);
    duty = ms_signal_to_pwm(vC, 0.1f, 0.25f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(17), PWM_CHAN_B, duty); 
    ******************************/
    
    /***********************************************/
    // Para o exemplo setup_rl_simple()
    float V1, iR1;

    V1 = ms_get_node_voltage(c, 1);
    // Normaliza 0–3.3 V para 0–1
    uint16_t duty = ms_signal_to_pwm(V1, 1.0f/3.3f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(16), PWM_CHAN_A, duty);
    
    //iR1 = ms_get_element_current(c, 2);
    iR1 = ms_get_resistor_current(c, 2);
    duty = ms_signal_to_pwm(iR1, 1.00f, 0.5f, PWM_WRAP);
    pwm_set_chan_level(pwm_gpio_to_slice_num(17), PWM_CHAN_B, duty);  
     /**********************************************/
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
    ms_set_source_external(c, Vsrc, adc_in, 3.3f, 0.0f); // ADC normalizado * 3.3V
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
    ms_add_resistor(c, 2, 3, 0.1f);   // 0.1Ω

    // C entre nó3 e terra
    ms_add_capacitor(c, 3, 0, 33e-6f); // 33µF

    // Fonte de tensão controlada pelo ADC
    int Vsrc = ms_add_voltage_source(c, 1, 0, 0.0f);
    ms_set_source_external(c, Vsrc, adc_in, 3.3f, 0.0f); // ADC normalizado * 3.3V
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
    ms_set_source_sine(c, Vsine, 0.0f, 1.0f, 60.0f, 0.0f);
    // offset=0, amplitude=1 V, freq=60 Hz, fase=0

    // Circuito RL em série:
    // Nó1 -> R -> Nó2 -> L -> Terra
    ms_add_resistor(c, 1, 2, 1.00f);    // R = 1.0 Ω
    ms_add_inductor(c, 2, 0, 0.01f);     // L = 10.0 mH

    // Resistor de carga para terra (evita nó flutuante)
    ms_add_resistor(c, 2, 0, 50.0e3f);   // R = 50 kΩ
    // resistor de estabilização
    ms_add_resistor(c, 3, 0, 1.0e6f); // resistor de estabilização
}