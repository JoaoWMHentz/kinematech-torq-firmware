# Kinematech Torq ESC Family

![Build Status](https://img.shields.io/badge/build-passing-brightgreen) ![License](https://img.shields.io/badge/license-custom-blue)

Família de ESCs de alta potência desenvolvida para robótica, veículos elétricos e aplicações industriais.

## Overview
A linha **Torq ESC** nasceu para entregar controle vetorial de motores BLDC com a mesma ergonomia das APIs SimpleFOC, porém dimensionada para correntes elevadas e integração industrial. O projeto combina uma placa lógica baseada em STM32G473 com um power stage compacto e redundâncias de proteção para garantir confiabilidade em aplicações de mobilidade, pesquisa e prototipagem rápida.

## Hardware Design
- MCU principal **STM32G473** com unidade de cálculo em ponto flutuante e periféricos avançados de temporização.
- **TIM1** operando em modo center-aligned para gerar PWM trifásico com deadtime programável.
- Etapa de potência com **MOSFETs N-channel** de baixa Rds(on), driver de gate isolado e barramento de capacitores de alta corrente.
- Sensoriamento integrado de corrente, tensão de barramento e temperatura via ADCs injetados.
- Proteções de sobrecorrente, subtensão/sobretensão, monitoramento térmico e circuito de brake/comp.

## ⚡ Electrical Specs
| Parâmetro | Valor | Notas |
| --- | --- | --- |
| Corrente contínua | 150 A | Com refrigeração forçada e barramento de cobre.
| Corrente de pico (10 s) | 250 A | Limitada por firmware e monitoramento térmico.
| Tensão de barramento | 18 V – 60 V | Compatível com packs 6S–14S Li-ion/LiPo.
| Frequência PWM | 20 kHz (center-aligned) | Ajustável via CubeMX.
| Deadtime | 250 ns – 500 ns | Programável em TIM1.
| Sampling ADC | Síncrono ao PWM | Modo injected com alinhamento a cada ciclo.
| Temperatura de operação | -20 °C a 80 °C | Monitoramento via NTCs em phases e barramento.

## 🖥️ Firmware
- Organização modular seguindo o estilo **SimpleFOC Codex**, com camadas para drivers PWM, sensores, matemática (Clarke/Park, SVPWM) e controladores FOC/Open-loop.
- Estrutura baseada em **STM32CubeMX**, mantendo arquivos gerados separados e abstrações próprias em `Core/Inc` e `Core/Src`.
- Módulos implementados incluem: `PwmDriverTim1`, blocos de sensoriamento de corrente/ângulo, controladores `BLDCMotor`, além de utilitários de PID e filtros.
- Fluxo de inicialização via `ESC_Main_Init()` conecta drivers, sensores e limites de tensão/corrente; o laço principal chama `LoopFOC()` e `Move()` com setpoints provenientes de CAN/UART ou scripts de teste.
- Política de zero alocação dinâmica em tempo real, monitoramento de faults e telemetria básica (θ, ω, Vbus, Iq, temperatura).

## Testes de Validação
- Ensaios de PWM em bancada com carga resistiva e análise de duty nos canais TIM1.
- Testes com **lâmpada automotiva H4** para validar limitação de corrente e resposta térmica.
- Verificação de integridade mecânica/ground spring no barramento de potência e aperto dos terminais.
- Sequência de comissionamento: autoteste de sensores, habilitação suave (`Enable()`), rampas de tensão e ensaio de torque constante.

## Estrutura do Repositório
```text
.
├── Core
│   ├── Inc
│   │   ├── config
│   │   ├── drivers
│   │   ├── foc
│   │   ├── math
│   │   └── sensors
│   ├── Src
│   │   ├── config
│   │   ├── drivers
│   │   ├── foc
│   │   ├── math
│   │   └── sensors
│   └── Startup
├── Drivers
│   ├── CMSIS
│   └── STM32G4xx_HAL_Driver
├── cmake
└── kinematech-torq-firmware.ioc
```

## 🚀 Roadmap
- [x] Revisão de hardware rev. B e testes elétricos básicos.
- [x] Porta inicial do framework SimpleFOC para STM32G473.
- [ ] Implementar sensoriamento de corrente trifásico com calibração automática.
- [ ] Integrar comunicação CAN FD e perfil UAVCAN.
- [ ] Publicar planilha completa de especificações e limites térmicos.
- [ ] Liberar exemplos de controle de velocidade/torque em modo campo alinhado.

## Fotos
![Torq ESC Control Board](docs/images/torq-esc-control.jpg)
![Torq ESC Power Stage](docs/images/torq-esc-power.jpg)
![Esquemático Principal](docs/images/torq-esc-schematic.png)

## Contribuição
1. Faça um fork do repositório.
2. Crie uma branch para sua feature ou correção (`git checkout -b feature/nome`).
3. Garanta que os testes e builds passem.
4. Abra um Pull Request descrevendo suas mudanças e validando o checklist de PR.

## Licença
Projeto disponibilizado para uso **educacional e de pesquisa**. Licenciamento **comercial sob demanda** mediante acordo com a Kinematech. Entre em contato para mais detalhes.

