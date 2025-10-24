# ⚙️ Configuração de Prioridades NVIC

## 🚨 IMPORTANTE: Ajustar no CubeMX

Após implementar o código, você **DEVE** ajustar as prioridades no **STM32CubeMX** para evitar conflitos e garantir que USB CDC funcione:

---

## 📋 Prioridades Corretas

Abra o arquivo `.ioc` no CubeMX e vá em **NVIC Settings**:

```
Peripheral                  | Preemption Priority | Sub Priority
----------------------------|---------------------|-------------
TIM1 Update (PWM FOC)       |         0           |      0
TIM8 CC (Hall Interface)    |         2           |      0
USB OTG FS                  |         5           |      0
```

---

## 🎯 Por que essas prioridades?

### **TIM1 (Prioridade 0 - Máxima)**
- Gera PWM para FOC
- Timing crítico: não pode ter jitter!
- ISR futura será a mais rápida (5-10μs)

### **TIM8 (Prioridade 2 - Alta)**
- Hall sensor
- Importante, mas pode esperar TIM1 terminar
- ISR ultra rápida (1-2μs): só captura dados

### **USB (Prioridade 5 - Background)**
- Comunicação não-crítica
- Pode ser interrompida por TIM1 e TIM8
- Processa em "tempo livre"

---

## 🔧 Como aplicar no CubeMX

1. Abra `kinematech-torq-firmware.ioc`
2. Clique na aba **Configuration**
3. Clique em **System Core** → **NVIC**
4. Ajuste as prioridades:
   - `TIM1 update interrupt`: **0** / **0**
   - `TIM8 capture compare interrupt`: **2** / **0**
   - `USB OTG FS global interrupt`: **5** / **0**
5. Salve e **regenere o código**

---

## ⚠️ Regra de Ouro

**Menor número = Maior prioridade**

```
Prioridade 0 > Prioridade 1 > Prioridade 2 > ... > Prioridade 15
```

**ISR de prioridade menor NUNCA interrompe ISR de prioridade maior!**

Exemplo:
- Se TIM1 (prio 0) está rodando, TIM8 (prio 2) **espera**
- Se TIM8 (prio 2) está rodando, USB (prio 5) **espera**
- Se USB (prio 5) está rodando, TIM1 ou TIM8 **interrompem imediatamente**

---

## 🐛 Se USB não funcionar

1. Verifique se **TIM8 priority < 5** (deve ser 2!)
2. Nunca chame `CDC_Transmit_FS()` dentro de ISR
3. Garanta que ISRs sejam **rápidas** (<10μs)

---

## ✅ Checklist Final

- [ ] TIM1 priority = 0
- [ ] TIM8 priority = 2  
- [ ] USB priority = 5
- [ ] Código regenerado pelo CubeMX
- [ ] Testado: Hall funciona + USB responde

---

**Após ajustar, recompile e teste!** 🚀
