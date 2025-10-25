# TIM1 - Configuração para FOC @ 20kHz

## Resumo da Implementação

✅ **Open-Loop FOC implementado com execução por interrupção**

### Arquitetura Final

```
TIM1 Update ISR (20kHz, prioridade 0) → OpenLoopFOC_Update()
    ↓
Calcula SVPWM (Clarke/Park inversa)
    ↓
Atualiza PWM TIM1 CH1/CH2/CH3 (duty cycles)
    ↓
Motor gira em malha aberta
```

---

## Configuração TIM1 no STM32CubeMX

### 1. Clock Configuration
- **APB2 Timer Clock:** 170 MHz (já configurado)
- **Prescaler:** 0 (usa clock full 170MHz)
- **ARR (Auto-Reload):** `170000000 / 20000 - 1 = 8499`
  - Frequência PWM = 20kHz
  - Resolução: 8500 steps (suficiente para SVPWM)

### 2. TIM1 - Mode & Configuration

#### Clock Source
- **Clock Source:** Internal Clock

#### Channel Configuration
- **CH1, CH2, CH3:** PWM Generation CHx CHxN
  - Mode: `PWM mode 1`
  - Pulse: `4250` (50% inicial - motor parado)
  - Output Compare Preload: **Enable**
  - Fast Mode: Disable
  - CH Polarity: High
  - CHN Polarity: High

#### PWM Generation
- **Counter Mode:** Up
- **Prescaler:** `0`
- **Counter Period (ARR):** `8499`
- **Internal Clock Division:** No Division
- **Repetition Counter:** `0`
- **Auto-reload preload:** Enable

#### Break and Dead Time
- **Dead Time:** `170` (1μs @ 170MHz)
  - Dead time = 170 / 170MHz = 1μs
  - Protege MOSFETs durante comutação
- **Break Input:** Enable (opcional - proteção hardware)
- **Break Polarity:** High
- **Automatic Output Enable:** Enable

#### Trigger Output (TRGO)
- **Trigger Event Selection:** Update Event
  - Pode ser usado para sincronizar ADC (futuro)

### 3. NVIC Settings

#### Interrupções a Habilitar
- ✅ **TIM1 update interrupt and TIM16 global interrupt**
  - **Priority:** `0` (MÁXIMA - FOC crítico)
  - **Subpriority:** `0`

#### Prioridades Finais (verificar)
```
TIM1_UP (FOC):     0  ← MÁXIMA
TIM8_CC (Hall):    2
USB_LP:            5
TIM2 (SysTick):   15  ← MÍNIMA
```

### 4. GPIO Configuration

#### Pinos PWM (exemplo para placa custom)
| Pino   | Função      | Modo        |
|--------|-------------|-------------|
| PA8    | TIM1_CH1    | AF6 (TIM1)  |
| PA7    | TIM1_CH1N   | AF6 (TIM1)  |
| PA9    | TIM1_CH2    | AF6 (TIM1)  |
| PB0    | TIM1_CH2N   | AF6 (TIM1)  |
| PA10   | TIM1_CH3    | AF6 (TIM1)  |
| PB1    | TIM1_CH3N   | AF6 (TIM1)  |
| PA12   | TIM1_BKIN   | AF6 (Break) |

**IMPORTANTE:** Verificar pinout da sua placa! Ajustar conforme esquemático.

---

## Após Gerar Código (.ioc → Generate)

### 1. Descomentar no `main.c` (USER CODE BEGIN 2):

```c
// Habilitar PWM complementar TIM1
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

// Habilitar interrupção Update @ 20kHz
HAL_TIM_Base_Start_IT(&htim1);
```

### 2. Testar FOC

```c
// No main loop ou via comando USB:
OpenLoopFOC_Enable(&foc_driver, 1);           // Habilitar driver
OpenLoopFOC_SetVelocity(&foc_driver, 500.0f); // 500 eRPM
```

### 3. Monitorar via USB

Adicionar ao telemetria (modificar `usb_communication.c`):
```c
telemetry.foc_angle = foc_driver.state.angle_openloop;
telemetry.foc_isr_count = foc_driver.state.isr_counter;
telemetry.duty_a = foc_driver.state.duty_a;
```

---

## Cálculos de Performance

### Timing Budget @ 20kHz
- **Período total:** 50μs
- **Execução ISR TIM1:** ~8-15μs (estimado)
  - Incremento ângulo: ~0.5μs
  - cos/sin (FPU): ~2μs
  - Transformadas: ~3μs
  - SVPWM: ~4μs
  - Atualizar CCR: ~2μs
- **Margem segura:** ~35μs para outras ISRs

### CPU Load
- FOC: 20kHz × 15μs = **30% CPU**
- Hall: ~100Hz × 2μs = **0.02% CPU**
- USB: ~100Hz × 50μs = **0.5% CPU**
- **Total: ~31% CPU @ 170MHz** ✅ Muito espaço para expansão

---

## Validação

### Checklist Pré-Energização
- [ ] Dead-time configurado (mín. 500ns)
- [ ] Break input conectado (opcional mas recomendado)
- [ ] PWM inicia em 50% (motor parado)
- [ ] Driver desabilitado por padrão
- [ ] Limitador de corrente funcionando (futuro - INA240)

### Testes Incrementais
1. **Bancada (sem motor):** Verificar PWM com osciloscópio
   - CH1, CH2, CH3 defasados 120°
   - Dead-time visível
   - Frequência = 20kHz
2. **Motor sem carga:** 100-500 eRPM
3. **Motor com carga:** Rampa até velocidade nominal

---

## Próximos Passos (após TIM1 funcionar)

1. **Closed-Loop FOC:** Usar Hall para fechar malha de velocidade
2. **Current Sensing:** Integrar INA240 + ADC via DMA
3. **Proteções:** Overcurrent, overvoltage, Hall fault
4. **Comandos USB:** Parser para controle remoto
5. **Auto-calibração:** Determinar sequência Hall automaticamente

---

## Troubleshooting

### Motor não gira
- Verificar sequência de fases (A-B-C invertido?)
- Conferir se PWM está ativo (LED nos gates?)
- Testar com tensão baixa primeiro (3V)

### Motor vibra/trava
- Dead-time muito longo ou curto
- Sequência SVPWM incorreta
- Velocidade muito baixa (< 100 eRPM dificulta open-loop)

### ISR demora muito (>50μs)
- Desabilitar printf/USB dentro de ISR
- Verificar se FPU está habilitada (CubeMX → FPU)
- Usar tabelas lookup para sin/cos (otimização futura)
