# PoC: Serviço de Temperatura por Localização

_Por: Cleber Akira Nakandakare_

- [PoC: Serviço de Temperatura por Localização](#poc-serviço-de-temperatura-por-localização)
  - [Introdução](#introdução)
    - [Escopo da Atividade](#escopo-da-atividade)
  - [Arquitetura da Solução](#arquitetura-da-solução)
    - [Conceito: Serviço de Temperatura sob Demanda](#conceito-serviço-de-temperatura-sob-demanda)
    - [Diagrama de Blocos](#diagrama-de-blocos)
  - [Implementação](#implementação)
    - [1. Coleta de Dados (Dispositivos de Borda)](#1-coleta-de-dados-dispositivos-de-borda)
    - [2. Interface com Cliente (Ubidots)](#2-interface-com-cliente-ubidots)
    - [3. Processamento Central e Lógica (Node-RED)](#3-processamento-central-e-lógica-node-red)
      - [A. Armazenamento de Dados](#a-armazenamento-de-dados)
      - [B. Cálculo da Temperatura (Geometria Analítica)](#b-cálculo-da-temperatura-geometria-analítica)
      - [C. Inicialização e Reset](#c-inicialização-e-reset)
    - [Mapeamento de Tópicos MQTT](#mapeamento-de-tópicos-mqtt)
    - [Ferramentas de Documentação](#ferramentas-de-documentação)
  - [Guia de Execução e Testes](#guia-de-execução-e-testes)
    - [Pré-requisitos e Download](#pré-requisitos-e-download)
    - [Inicialização do Node-RED (Docker)](#inicialização-do-node-red-docker)
    - [Simulação dos Sensores (ESP32/Wokwi)](#simulação-dos-sensores-esp32wokwi)
    - [Operação do Sistema (Ubidots)](#operação-do-sistema-ubidots)
    - [Reinicialização e Limpeza de Dados](#reinicialização-e-limpeza-de-dados)
  - [Validação dos Requisitos](#validação-dos-requisitos)
  - [Limitações e Trabalhos Futuros](#limitações-e-trabalhos-futuros)
  - [Conclusão](#conclusão)

---

## Introdução

Este documento detalha a solução desenvolvida para a "Atividade Entregável 2" do curso de **IoT em Sistemas Embarcados (2025/2026)**. O projeto consiste na criação de um sistema completo de telemetria e controle via MQTT, integrando dispositivos simulados, um *broker* de mensagens e um *dashboard* na nuvem.

O código-fonte completo e os artefatos do projeto estão disponíveis no repositório:
<https://github.com/cakira/IoT-Embarcados/tree/main/entregavel_2>.

### Escopo da Atividade

**Ferramentas:** Wokwi + HiveMQ (ou outro broker MQTT) + Plataforma Ubidots + Sistema desejado para
Integração

> 1. Criar device no Ubidots (2 a 4 variáveis de telemetria a critério do estudante) - Pode
>    reutilizar a Atividade 1 mas tente ser criativo melhorando o projeto anterior.
> 2. Configurar dashboard com widgets para o device criado - DICA: Como o Dashboard tem limite de
>    10 Variaveis, pode usar variaveis de contexto para aumentar esse limite. Veja Aulas da Semana 6
> 3. No Wokwi, publicar telemetria MQTT em algum Broker (MQTTx, HiveMQ, Mosquito, etc)
> 4. No Ubidots, configurar integração MQTT usando Token do device/variáveis para consumir dados do
>    Broker (via bridge, plugin, ou fluxo suportado pela plataforma, pode ser criado um outro
>    sistema com ESP32 como retransmissor como ex. da Semana 7).
> 5. Validar se os dados do device estão atualizando no dashboard do Ubidots - Gere um relatorio
>    (pode ser capturado do monitor serial) da origem dos dados e comparar com o resultado final.
>    Acrescente na documentação.
> 6. Inserir print do dashboard atualizado na documentação
> 7. Criar uma documentação adequada mostrando um rascunho simples da arquitetura, objetivo do
>    projeto, descrição geral do sistema, explicação de cada componente, seu fluxo de interação
>    entre eles, descrição das variaveis de publicação e subscrição e conclusoes. Faça um documento
>    formal com capa, titulo, indices, etc.
> 8. Fazer um pequeno video de poucos minutos mostrando o funcionamento dos sistemas, pode usar
>    captura de tela.

## Arquitetura da Solução

Para atender aos requisitos de forma didática e coerente com uma aplicação real, foi concebida uma arquitetura onde o Ubidots não apenas espelha dados brutos, mas atua como cliente de um serviço de valor agregado.

### Conceito: Serviço de Temperatura sob Demanda

O sistema simula um serviço onde o usuário solicita a temperatura estimada para uma coordenada geográfica específica. Diferente de um monitoramento passivo, o sistema opera da seguinte forma:

1.  Sensores distribuídos enviam dados brutos para um *broker* MQTT.
2.  O usuário seleciona uma localização no Ubidots.
3.  O sistema processa a requisição, realiza uma **interpolação espacial** baseada nos dados dos sensores vizinhos e retorna a temperatura estimada para aquele ponto.
4.  O serviço contabiliza um "custo" financeiro por requisição, simulando um modelo de cobrança.

Essa abordagem justifica a existência de um processamento intermediário (via Node-RED), evitando que o Ubidots apenas replique dados que o *broker* já possui.

### Diagrama de Blocos

A solução integra três ESP32 simulados (entrada de dados), a plataforma Node-RED (processamento central) e o Ubidots (interface final). O fluxo de comunicação é integralmente baseado no protocolo MQTT.

| ![Diagrama em blocos do Serviço de Temperatura por Localização](Block_diagram_1.png) |
| :----------------------------------------------------------: |
|   _Figura 1: Diagrama em blocos da arquitetura proposta_    |

O Node-RED foi escolhido para realizar a integração e o processamento dos dados pela sua flexibilidade e facilidade didática na criação de fluxos de dados (_Low-code_).

## Implementação

O sistema divide-se em três macro-componentes: **Coleta de Dados**, **Interface com Cliente** e **Aplicação Node-RED**.

### 1. Coleta de Dados (Dispositivos de Borda)

A camada de borda é composta por três dispositivos ESP32 simulados no Wokwi. Cada dispositivo representa uma estação meteorológica fixa, enviando temperatura (lida via sensor NTC) e suas coordenadas geográficas a cada 2 segundos.

Como o simulador não possui módulo GPS, as localizações foram definidas via *firmware*, correspondendo a pontos reais de instituições de ensino/pesquisa:

| ID | Localização | Latitude | Longitude |
| :- | :---------- | :------- | :-------- |
| 0  | CPQD        | -22.8162 | -47.0451  |
| 1  | FIAP        | -22.9027 | -47.0563  |
| 2  | CTI         | -22.8517 | -47.1284  |

A identificação de cada sensor é configurada fisicamente através de um *DIP-switch* (chaves 7 e 8) lido apenas na inicialização do dispositivo, conforme ilustrado a seguir:

| ![ESP32 usado como sensor de temperatura](ESP32_temperature_sensor.png) |
| :----------------------------------------------------------: |
|         _Figura 2: Configuração do ID via DIP-switch_          |

### 2. Interface com Cliente (Ubidots)

O Ubidots atua como a interface de solicitação do serviço. Foi criado um dispositivo virtual denominado _"Temperature Requester"_ contendo as variáveis de controle (inputs do usuário) e de resposta (outputs do sistema).

**Variáveis de Controle:**
* `req_lat` e `req_lon`: Coordenadas alvo da requisição.
* `req_run`: Gatilho (_trigger_) para disparar o cálculo.

**Variáveis de Resposta:**
* `cost`: Valor acumulado do serviço.
* `position`: Variável composta que armazena a temperatura estimada e o contexto geográfico para plotagem no mapa.

**Customizações do Dashboard:**
* **Sliders:** Configurados com limites precisos (ex: Lat -22.81° a -22.91°) e orientação (vertical para latitude, horizontal para longitude) para facilitar a seleção geográfica.
* **Mapa:** Implementado seguindo a documentação avançada de *widgets* do Ubidots. Para enriquecer a visualização, foram adicionados marcadores estáticos (Device CPQD e Device FIAP) servindo como referência visual para o usuário.

| ![Dashboard no Ubidots](Ubidots_dashboard.png) |
| :--------------------------------------------: |
|   _Figura 3: Interface de requisição e mapa_   |

### 3. Processamento Central e Lógica (Node-RED)

O Node-RED é o núcleo do sistema, responsável por receber os dados, executar a lógica e devolver as respostas. Ele assina os tópicos dos sensores e do Ubidots, executa o cálculo matemático e publica os resultados.

Foi desenvolvido um *dashboard* administrativo no Node-RED para monitoramento dos dados brutos e gestão do sistema (reset de custos e limpeza de dados).

| ![Diagrama de fluxo no Node-RED](Node_red_flow.png) |
| :-------------------------------------------------: |
|          _Figura 4: Fluxo completo no Node-RED_           |

A lógica foi segmentada em três blocos funcionais:

#### A. Armazenamento de Dados
Ao receber dados MQTT dos sensores, o fluxo extrai o ID do dispositivo e armazena o objeto JSON (`lat`, `lon`, `temp`) em uma variável de fluxo global `sensorData`. Isso garante que o sistema sempre tenha o "estado atual" de todos os sensores disponível para cálculo.

#### B. Cálculo da Temperatura (Geometria Analítica)
Quando o usuário aciona o gatilho no Ubidots, o nó `Calc temperature` executa a interpolação. A lógica baseia-se na **Equação Geral do Plano** determinada por três pontos no espaço 3D (onde X=Latitude, Y=Longitude, Z=Temperatura).

O algoritmo realiza os seguintes passos:
1.  Recupera a última leitura válida dos sensores 0, 1 e 2.
2.  Calcula os vetores diretores entre os pontos.
3.  Obtém o vetor normal através do produto vetorial.
4.  Resolve a equação para encontrar Z (temperatura) nas coordenadas (X, Y) solicitadas pelo usuário.
5.  Incrementa o contador de custo ($0,01 por requisição).

_Nota: O trecho de código JavaScript responsável por essa lógica pode ser consultado diretamente no arquivo `flows.json` ou na documentação original._

#### C. Inicialização e Reset
Fluxos auxiliares garantem a limpeza das variáveis globais e o reset dos contadores via botões administrativos, enviando feedback visual ao painel de controle.

### Mapeamento de Tópicos MQTT

A comunicação entre os componentes segue a topologia definida na tabela abaixo.

| Origem | Destino | Tópico | Descrição |
| :--- | :--- | :--- | :--- |
| **Sensor ESP32** | Node-RED | `.../sensor/<id>` | Telemetria bruta (Lat, Lon, Temp) |
| **Ubidots (Slider)** | Node-RED | `.../req_lat/lv` | Latitude solicitada |
| **Ubidots (Slider)** | Node-RED | `.../req_lon/lv` | Longitude solicitada |
| **Ubidots (Botão)** | Node-RED | `.../req_run/lv` | Gatilho de execução |
| **Node-RED** | Ubidots (Mapa) | `.../temperature-requester` | JSON com temperatura e contexto geo |
| **Node-RED** | Ubidots (Display)| `.../cost` | Valor monetário acumulado |

> **Legendas:**
> * Prefixos MQTTx: `IoT-Embarcados/akira/entrega/2`
> * Prefixos Ubidots: `/v1.6/devices/temperature-requester`

### Ferramentas de Documentação
Este relatório foi elaborado em Markdown no VS Code, utilizando os plugins "Markdown All in One" para formatação e "Markdown PDF" para exportação. Os diagramas foram produzidos via <https://www.drawio.com/>.

---

## Guia de Execução e Testes

Siga os procedimentos abaixo para reproduzir o ambiente e validar o funcionamento.

### Pré-requisitos e Download
* Docker instalado.
* VS Code com extensão Wokwi e PlatformIO.
* Conta no Ubidots (Educational/Stem).

Clone o repositório do projeto:
;;;bash
git clone https://github.com/cakira/IoT-Embarcados/
cd IoT-Embarcados/entregavel_2
;;;

### Inicialização do Node-RED (Docker)

1.  **Instalação de dependências (Primeira execução):**
    Execute o comando abaixo para instalar o *FlowFuse Dashboard* no volume persistente:
    ;;;bash
    docker run -it --rm -p 1880:1880 \
        --mount type=bind,src=$PWD/data_node_red,dst=/data \
        --entrypoint /bin/bash \
        nodered/node-red:4.1.3 \
        -c "cd /data && npm install @flowfuse/node-red-dashboard@1.30.2"
    ;;;

2.  **Execução do Serviço:**
    Inicie o container Node-RED:
    ;;;bash
    docker run -it --rm -p 1880:1880 \
        --mount type=bind,src=$PWD/data_node_red,dst=/data \
        nodered/node-red:4.1.3
    ;;;

3.  Acesse o painel administrativo em: <http://127.0.0.1:1880/dashboard>.

### Simulação dos Sensores (ESP32/Wokwi)

1.  No VS Code, abra a pasta do projeto.
2.  Configure o **Sensor 0**: Ajuste o *DIP-Switch* simulado para `OFF, OFF`.
3.  Inicie a simulação (Play). Verifique no console serial a conexão MQTT e envio de dados.
4.  Repita o processo para o **Sensor 1** (`OFF, ON`) e **Sensor 2** (`ON, OFF`), preferencialmente abrindo instâncias simultâneas do VS Code ou alternando a execução.
5.  *Verificação:* Confirme se os três sensores aparecem na tabela do *dashboard* do Node-RED.

### Operação do Sistema (Ubidots)

Devido às limitações de compartilhamento da conta gratuita do Ubidots, os testes devem ser feitos replicando os *widgets* descritos na seção de implementação.

1.  No *dashboard* do Ubidots, defina uma coordenada usando os *sliders* de Latitude e Longitude.
2.  Clique no botão de gatilho.
3.  **Resultado Esperado:**
    * O mapa atualiza o marcador para a nova posição.
    * O *widget* de temperatura exibe o valor calculado.
    * O contador de custo é incrementado.

| ![Resultado da requisição no Ubidots](Results_ubidots_dashboard_1.png) |
| :----------------------------------------------------------: |
|           _Figura 5: Visualização do resultado no mapa_            |

### Reinicialização e Limpeza de Dados
Para validar as funções administrativas, acesse o *dashboard* do Node-RED e utilize os botões **Reset Sensor Data** e **Reset Request Counter**. Verifique se a tabela é limpa e se o custo no Ubidots retorna a zero.

---

## Validação dos Requisitos

Abaixo, a matriz de rastreabilidade entre o enunciado da atividade e a solução entregue:

| Status | Tarefa do Enunciado | Observação |
| :---: | :--- | :--- |
| ✅ | Criar device no Ubidots (2-4 variáveis) | Criado device "Requester" com 5 variáveis. |
| ✅ | Configurar dashboard com widgets | Sliders, Mapa, Indicadores e Switch implementados. |
| ✅ | Publicar telemetria MQTT via Wokwi | 3 instâncias de ESP32 simuladas. |
| ✅ | Integração MQTT (Broker → Ubidots) | Realizada via Node-RED (Aplicação Central). |
| ✅ | Validar atualização dos dados | Comprovada via comparação de logs e screenshots. |
| ✅ | Inserir prints do dashboard | Figuras incluídas ao longo do documento. |
| ✔️ | Documentação formal da arquitetura | Documento estruturado (sem capa formal). |
| ✅ | Vídeo de demonstração | Entregue em anexo/separado. |

---

## Limitações e Trabalhos Futuros

Como Prova de Conceito (PoC), o sistema atinge seus objetivos, mas apresenta oportunidades de evolução para um produto final:

* **Segurança:** Implementação de MQTTS (SSL/TLS) e autenticação de usuários para proteger o acesso aos dados e ao controle.
* **Escalabilidade e Algoritmos:** Substituição do cálculo de plano (limitado a 3 pontos) por algoritmos de interpolação que suportem *n* sensores, como a Ponderação pelo Inverso da Distância (IDW).
* **Precisão Geográfica:** Adoção de fórmulas que considerem a curvatura da Terra (fórmula de Haversine ou elipsoide) para aumentar a precisão em distâncias maiores.
* **Resiliência:** Tratamento de erros de comunicação com o *broker* e feedback visual no Ubidots caso o serviço de cálculo esteja indisponível.

## Conclusão

Este projeto cumpriu integralmente os requisitos da disciplina de IoT, demonstrando a integração prática entre dispositivos de borda, lógica de nuvem e interface de usuário.

O uso do Node-RED como centralizador da lógica provou-se uma escolha arquitetural acertada, permitindo abstrair a complexidade matemática da interpolação e controlar a regra de cobrança, enquanto o Ubidots foi utilizado naquilo que oferece de melhor: visualização de dados e interação com o usuário final. Além da validação técnica, o desenvolvimento proporcionou domínio sobre a orquestração de containers Docker e fluxos MQTT avançados.
