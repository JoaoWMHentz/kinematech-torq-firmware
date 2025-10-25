# Relatório de Validação - TIM1 FOC @ 20kHz

**Data:** 24 de outubro de 2025  
**Status:** ⚠️ **CORREÇÕES CRÍTICAS NECESSÁRIAS**

---

## ❌ PROBLEMAS CRÍTICOS ENCONTRADOS

### 1. TIM1 Counter Mode Incorreto (ALTA PRIORIDADE)

**Configuração atual (.ioc):**
```
TIM1.CounterMode=TIM_COUNTERMODE_CENTERALIGNED1
```

**Problema:**
- Center-aligned mode conta UP e DOWN, dobrando a frequência efetiva
- Com ARR=8499, a frequência será ~40kHz ao invés de 20kHz
- ISR será chamada 2x mais rápido que o esperado (25μs ao invés de 50μs)

**Correção obrigatória:**
```
TIM1.CounterMode=TIM_COUNTERMODE_UP
```

**Como corrigir:**
1. Abrir `kinematech-torq-firmware.ioc` no STM32CubeMX
2. TIM1 → Parameter Settings → Counter Mode → **Up**
3. Salvar e gerar código

---

### 2. Dead-time PERIGOSAMENTE BAIXO (CRÍTICO!)

**Configuração atual (.ioc):**
```
TIM1.DeadTime=5
```

**Cálculo do dead-time atual:**
```
Dead-time = 5 ticks @ 170MHz = 29 nanosegundos
```

**⚠️ RISCO DE HARDWARE:**
- MOSFETs high-side e low-side podem conduzir simultaneamente
- **Shoot-through** irá destruir os MOSFETs em poucos segundos
- Mínimo seguro para MOSFETs típicos: **500ns - 1μs**

**Correção obrigatória:**
```
TIM1.DeadTime=170  // 1μs @ 170MHz
```

**Cálculo correto:**
```
Dead-time = 170 ticks @ 170MHz = 1000 nanosegundos (1μs) ✅
```

**Como corrigir:**
1. Abrir `kinematech-torq-firmware.ioc` no STM32CubeMX
2. TIM1 → Parameter Settings → Dead Time → **170**
3. Salvar e gerar código

---

### 3. PWM Complementar - Código Corrigido ✅

**Problema original:**
O código não estava iniciando os canais complementares (CHxN).

**Correção aplicada automaticamente:**
```c
// ANTES (INCORRETO - faltavam canais complementares):
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

// DEPOIS (CORRETO - canais complementares habilitados):
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // CH1N

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // CH2N

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // CH3N

HAL_TIM_Base_Start_IT(&htim1);  // ISR @ 20kHz
```

**Status:** ✅ Código corrigido no arquivo `main.c`

---

## ✅ CONFIGURAÇÕES CORRETAS VALIDADAS

### TIM1 - Configuração PWM

| Parâmetro | Valor Atual | Status | Observação |
|-----------|-------------|--------|------------|
| Prescaler | 0 | ✅ | Clock full 170MHz |
| ARR (Period) | 8499 | ✅ | 170MHz / 8500 = 20kHz |
| Pulse CH1/2/3 | 4250 | ✅ | 50% inicial (motor parado) |
| Auto-reload Preload | ENABLE | ✅ | Evita glitches |
| Automatic Output | ENABLE | ✅ | Necessário para break |
| OffState Run/IDLE | ENABLE | ✅ | Segurança em falha |

### TIM1 - NVIC (Interrupções)

| IRQ | Prioridade | Status | Observação |
|-----|------------|--------|------------|
| TIM1_UP_TIM16_IRQn | 0 | ✅ | Máxima prioridade (FOC) |
| TIM8_CC_IRQn (Hall) | 2 | ✅ | Pode preemptar USB |
| USB_LP_IRQn | 5 | ✅ | Baixa prioridade |
| TIM2_IRQn (SysTick) | 15 | ✅ | Mínima prioridade |

### TIM1 - Pinos GPIO

| Pino | Função | Configuração | Status |
|------|--------|--------------|--------|
| PA8 | TIM1_CH1 (PA_HIN) | PWM, PULLDOWN, VERY_HIGH | ✅ |
| PB13 | TIM1_CH1N (PA_LIN) | PWM, PULLDOWN, VERY_HIGH | ✅ |
| PA9 | TIM1_CH2 (PB_HIN) | PWM, PULLDOWN, VERY_HIGH | ✅ |
| PB14 | TIM1_CH2N (PB_LIN) | PWM, PULLDOWN, VERY_HIGH | ✅ |
| PA10 | TIM1_CH3 (PC_HIN) | PWM, PULLDOWN, VERY_HIGH | ✅ |
| PB15 | TIM1_CH3N (PC_LIN) | PWM, PULLDOWN, VERY_HIGH | ✅ |

### TIM8 - Hall Sensor Interface

| Parâmetro | Valor | Status |
|-----------|-------|--------|
| Mode | Hall Sensor Interface | ✅ |
| Prescaler | 169 | ✅ (1MHz tick) |
| IC1 Polarity | RISING | ✅ |
| Pinos | PB6, PB8, PB9 | ✅ |

### RCC - Clock Tree

| Clock | Frequência | Status |
|-------|------------|--------|
| SYSCLK | 170 MHz | ✅ |
| APB1 Timer | 170 MHz | ✅ |
| APB2 Timer | 170 MHz | ✅ |
| USB (HSI48) | 48 MHz | ✅ |
| HSE | 24 MHz | ✅ |

---

## 📋 CHECKLIST DE CORREÇÕES

### Obrigatórias (antes de energizar motor):

- [ ] **CRÍTICO:** Alterar TIM1 Counter Mode para **UP** (não Center-aligned)
- [ ] **CRÍTICO:** Alterar TIM1 Dead Time para **170** (1μs mínimo)
- [ ] Gerar código novamente no CubeMX
- [ ] Compilar e verificar sem erros
- [ ] ✅ Código main.c já corrigido (PWM complementar)

### Recomendadas:

- [ ] Adicionar Break Input (PA12) conectado a comparador de overcurrent
- [ ] Configurar ADC via DMA para INA240 (leitura de corrente)
- [ ] Adicionar watchdog (IWDG) para reset em travamento
- [ ] Implementar soft-start com rampa de tensão suave

---

## 🧪 PLANO DE TESTES PÓS-CORREÇÃO

### 1. Teste de Bancada (sem motor)

**Objetivo:** Validar sinais PWM com osciloscópio

**Medidas esperadas:**
- Frequência PWM: **20 kHz** (período = 50μs)
- Dead-time: **1μs** (entre falling edge HIN e rising edge LIN)
- Duty cycle inicial: **50%** em todos os canais
- Defasagem entre fases: **120°** (16.67μs @ 20kHz)

**Comandos de teste:**
```c
OpenLoopFOC_Enable(&foc_driver, 1);
OpenLoopFOC_SetVelocity(&foc_driver, 0.0f);  // Motor parado, apenas 50% PWM
```

### 2. Teste de Timing ISR

**Objetivo:** Verificar performance da ISR TIM1

**Adicionar ao código (temporário):**
```c
// No início de OpenLoopFOC_Update():
HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);

// No fim de OpenLoopFOC_Update():
HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
```

**Medida esperada:** Pulso LED = 8-15μs (tempo de execução ISR)

### 3. Teste Open-Loop (baixa velocidade)

**Objetivo:** Girar motor em malha aberta

**Sequência:**
```c
OpenLoopFOC_Enable(&foc_driver, 1);
OpenLoopFOC_SetVelocity(&foc_driver, 100.0f);  // 100 eRPM
HAL_Delay(2000);
OpenLoopFOC_SetVelocity(&foc_driver, 500.0f);  // 500 eRPM
```

**Esperado:**
- Motor gira suavemente sem travamentos
- Corrente < 5A (sem carga)
- Sem ruído excessivo

---

## 🔍 ANÁLISE DE PERFORMANCE

### Timing Budget @ 20kHz (50μs período)

| Componente | Tempo (μs) | % CPU |
|------------|------------|-------|
| ISR TIM1 (FOC) | ~12μs | 24% |
| ISR TIM8 (Hall) | ~2μs | 0.04% |
| USB Telemetry | ~50μs @ 20Hz | 0.1% |
| **Total estimado** | - | **~25%** |
| **Margem livre** | - | **~75%** ✅ |

### Validação de Frequências

**TIM1 PWM (após correção):**
```
Frequência = APB2_Timer_Clock / ((Prescaler + 1) × (ARR + 1))
          = 170 MHz / (1 × 8500)
          = 20.000 Hz ✅
```

**TIM1 PWM (ATUAL - INCORRETO):**
```
Modo Center-aligned dobra frequência:
          = 170 MHz / (1 × 8500 × 2)
          = 10.000 Hz (contador) → 20.000 Hz (edges) → ISR @ 40kHz ❌
```

**TIM8 Hall (tick rate):**
```
Tick Rate = 170 MHz / (169 + 1)
          = 1 MHz (1μs por tick) ✅
```

---

## 📝 RESUMO EXECUTIVO

### Status Atual

✅ **Arquitetura de software:** Correta (FOC em ISR, Hall em ISR separada)  
✅ **Prioridades NVIC:** Corretas (TIM1=0, TIM8=2, USB=5)  
✅ **Pinout GPIO:** Correto (6 pinos PWM complementares)  
✅ **Clock tree:** Correto (170MHz para timers)  
⚠️ **TIM1 Counter Mode:** INCORRETO (center-aligned ao invés de up)  
❌ **TIM1 Dead-time:** PERIGOSO (29ns ao invés de 1μs)  
✅ **Código main.c:** Corrigido (PWM complementar habilitado)

### Ação Imediata Requerida

**NÃO ENERGIZAR O MOTOR ATÉ CORRIGIR:**

1. Abrir `kinematech-torq-firmware.ioc`
2. TIM1 → Counter Mode → **Up**
3. TIM1 → Dead Time → **170**
4. Gerar código
5. Compilar e flash

**Após correções, o sistema estará pronto para testes.**

---

## 🆘 TROUBLESHOOTING

### Motor não gira após correções

- Verificar se `OpenLoopFOC_Enable(&foc_driver, 1)` foi chamado
- Verificar se velocidade > 0 (`OpenLoopFOC_SetVelocity()`)
- Medir PWM com osciloscópio (confirmar 20kHz)
- Verificar alimentação do driver (12V presente?)

### MOSFETs esquentam muito

- **DESLIGAR IMEDIATAMENTE**
- Possível shoot-through (dead-time ainda insuficiente)
- Verificar se dead-time = 170 no código gerado (`tim.c`)
- Aumentar dead-time para 340 (2μs) se persistir

### ISR demora muito (>50μs)

- Desabilitar USB_Comm_Printf() dentro de ISR
- Verificar se FPU está habilitada (Project Settings → Floating Point)
- Adicionar GPIO toggle para medir tempo real

### Motor vibra/trava

- Velocidade muito baixa (< 100 eRPM difícil em open-loop)
- Verificar sequência de fases (inverter duas se necessário)
- Aumentar tensão inicial (`foc_config.max_voltage`)

---

**Documento gerado automaticamente pela validação do sistema.**  
**Para dúvidas, consulte TIM1_FOC_SETUP.md**
