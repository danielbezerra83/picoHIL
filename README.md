# picoHIL
Raspberry pi pico based Hardware-In-the-Loop. 
![Diagrama](images/picoHIL.png)<br>
Este projeto consiste de:<br>
- Um conjunto de solvers para sistemas lineares do tipo A.x=B
- Um pequeno motor de análise nodal de circuitos (picoSPICE)
- Possibilidade de iteração com o mundo externo através de PWM como DAC e canais ADC para
interação com o circuito.
- Execução em tempo quasi-real do circuito (depende do número de nós e elementos).<br>
O objetivo é que o microcontrolador emule circuitos internamente, porém iteragindo com o mundo externo.<br>
A iteração é feita através de:<br>
- ADCs que servem como fontes de tensão ou corrente, com a respectiva adaptação de faixa de tensão e offset.<br>
- DACs via PWM+RC que servem para observar a diferença de potencial entre nós ou a corrente nos elementos.
- I/O (essencialmente input) cujo objetivo é acionar interruptores controlados (elementos SWITCH).<br>
O diagrama abaixo exibe a conexão com o pico2:<br>
![Diagrama](images/Raspberry-Pi-Pico-2-rp2350-low-res-pinout-mischianti.jpg)<br>
ADC: GP26 - ADC0, GP27 - ADC1, GP28 - ADC2<br>
PWM: GP16 - PWM0A, GP17 - PWM0B, GP18 - PWM1A, GP18 - PWM1B<br>
Nos pinos de PWM foram inseridas redes R-C (2k2 e 10nF) como filtro passa-baixa (formando um PWMDAC).<br>
Na placa que eu desenvolvi, optei por inserir um LM358 como seguidor em cada um dos 4 filtros PB.<br>

# Passo-A-Passo
Para rodar o picoHIL na Raspberry pi PICO (testado apenas na pico2 por enquanto):
- Instalar do VS Code + Extensão da Raspberry Pi Pico.
- Importar o projeto.
- Compilar e descarregar na placa.<br>
Exemplos de uso:
- Circuito RL com fonte senoidal interna associada à fonte externa imposta pelo sinal no ADC, corrente e tensão disponibilizadas nos PWMDAC.
- Circuito RLC com fonte senoidal externa imposta pelo ADC, corrente e tensão disponibilizadas nos PWMDAC.

Quaisquer dúvidas: daniel.bez.ifce@gmail.com

