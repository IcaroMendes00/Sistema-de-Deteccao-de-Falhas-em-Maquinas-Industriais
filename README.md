# Sistema de Detecção de Falhas em Máquinas Industriais

## 1. Introdução

A manutenção preditiva tem se tornado uma abordagem essencial na indústria para reduzir falhas inesperadas e otimizar o funcionamento de máquinas. Neste projeto, foi desenvolvido um sistema de detecção de falhas baseado em aprendizado de máquina embarcado, utilizando um microcontrolador **STM32F103CBT6**, um sensor **MPU6050** para aquisição de dados de vibração e o **NanoEdge AI Studio** para análise e inferência de anomalias.

O sistema permite a detecção automática de falhas mecânicas em motores industriais, analisando os padrões de vibração durante a operação. Os testes foram realizados utilizando dois coolers de CPU, um em condição normal e outro com falha simulada (adicionando peso às pás do cooler).

---

## 2. Objetivo

O objetivo deste projeto foi desenvolver um sistema embarcado inteligente para detecção de anomalias em máquinas industriais, utilizando um modelo de inteligência artificial treinado para identificar padrões normais e falhos de vibração.

Os principais objetivos incluem:

- Capturar dados de vibração em tempo real via **MPU6050**;
- Processar e analisar os dados usando um modelo de aprendizado de máquina **NanoEdge AI**;
- Detectar falhas automaticamente e sinalizar o estado do sistema via **UART e LED indicador**;
- Testar a eficiência do modelo com métricas de acurácia e tempo de resposta.

---

## 3. Metodologia

### 3.1. Configuração do Hardware

Para a coleta dos dados de vibração, foram utilizados:

- **STM32F103CBT6** – Microcontrolador para processar os dados e executar o modelo de IA;
- **MPU6050** – Sensor de aceleração de 3 eixos para capturar vibrações do motor;
- **Coolers de CPU** – Um em estado normal e outro com pesos nas pás para simular falhas;
- **LED Indicador (GPIOB, pino 2)** – Para indicar anomalias detectadas pelo sistema.

### 3.2. Aquisição e Processamento dos Dados

A coleta de dados foi realizada através do MPU6050, operando via **I2C a 400 kHz**, com o **STM32 rodando a 72 MHz**.

Os dados foram coletados no seguinte formato:

**linha 1** - x0 y0 z0 x1 y1 z1 ... x255 y255 z255 

**linha 2** - x0 y0 z0 x1 y1 z1 ... x255 y255 z255 

... 

**linha N** - x0 y0 z0 x1 y1 z1 ... x255 y255 z255


Foram coletadas 500 linhas de funcionamento normal e 500 linhas de funcionamento anômalo.

Durante a coleta, a comunicação serial do NanoEdge AI Studio apresentou falhas, sendo necessário o desenvolvimento de um script Python para armazenar os dados em formato CSV, remover amostras corrompidas e garantir a integridade das informações.

### 3.3. Treinamento do Modelo de IA

Os dados foram importados para o **NanoEdge AI Studio**, onde foram realizados os seguintes passos:

- Treinamento da IA utilizando a opção **Anomaly Detection**;
- Benchmarking para selecionar o melhor modelo de inferência;
- Geração da biblioteca **NanoEdge AI**, que foi integrada ao **STM32CubeIDE** para testes embarcados.

### 3.4. Implementação e Testes no STM32

Após o treinamento, a biblioteca foi embarcada no microcontrolador, e os testes foram conduzidos para avaliar:

- Tempo de resposta da inferência;
- Precisão na detecção de falhas;
- Comportamento do sistema em condições normais e anômalas.

---

## 4. Implementação no STM32

### 4.1. Configuração do LED para Indicação de Falhas

Para indicar anomalias detectadas, foi configurado um LED no **GPIOB, pino 2**, que:

- Fica desligado em funcionamento normal;
- Acende quando uma anomalia é detectada.

O sistema processa continuamente os dados do acelerômetro e realiza inferências utilizando a biblioteca NanoEdge AI. Quando um comportamento anômalo é identificado, o LED acende e os resultados são enviados via UART para monitoramento.

---

## 5. Resultados e Análises

### 5.1. Resultados Obtidos

- **Precisão:** O modelo atingiu 100% de acerto nos testes, superando a expectativa inicial de 95%;
- **Tempo de Resposta:** O esperado era 200 ns, mas o real foi de no máximo 2 segundos, devido ao processamento do STM32 e comunicação I2C;
- **Detecção em Tempo Real:** O sistema foi testado e conseguiu identificar todas as anomalias corretamente.

### 5.2. Observação

A latência pode ser reduzida otimizando o código para uso do DMA e interrupções.

---

## 6. Conclusão

Este projeto demonstrou que aprendizado de máquina embarcado pode ser uma solução eficaz para detecção de falhas em máquinas industriais.

### 6.1. Principais Conquistas

- Implementação funcional no **STM32F103CBT6**;
- Detecção precisa de anomalias em tempo real;
- Sistema autônomo e independente de servidores externos.

### 6.2. Melhorias Futuras

- Reduzir o tempo de inferência otimizando código e uso de DMA;
- Implementar filtragem de ruído para melhorar a qualidade dos dados;
- Testar o sistema com diferentes tipos de falhas mecânicas.

---

## 7. Como Executar o Projeto

### 7.1. Configuração do Ambiente

1. **Instale o STM32CubeIDE** e configure as bibliotecas HAL e NanoEdge AI;
2. **Conecte o STM32F103CBT6** e carregue o código-fonte;
3. **Compile e grave o firmware** no microcontrolador;
4. **Abra o monitor serial (115200 bps)** para visualizar os resultados.

Coleta e Processamento dos Dados (Python)

Para coletar novos dados e formatá-los no formato ideal em um arquivo .csv para o NanoEdge AI:

```bash
python csv_gen.py   # Coleta dados do MPU6050 via UART
```
