# PoC: Serviço de Temperatura por Localização

_Por: Cleber Akira Nakandakare_

## Introdução

Este documento apresenta a solução da atividade entregável 2 do curso de "IoT em Sistemas Embarcados
2025/2026".

O repositório original de projeto pode ser encontrado em:
<https://github.com/cakira/IoT-Embarcados/>, e os dados relativos ao entregável
2 estão nesse mesmo repositório, na pasta
[entregavel_2](https://github.com/cakira/IoT-Embarcados/tree/main/entregavel_2).

### Enunciado

#### Atividade 2 – Publicação de Telemetria do Dispositivo IoT (MQTT → Ubidots)

##### Descrição da tarefa:
**Ferramentas:** Wokwi + HiveMQ (ou outro broker MQTT) + Plataforma Ubidots + Sistema desejado para
Integração

1. Criar device no Ubidots (2 a 4 variáveis de telemetria a critério do estudante) - Pode
   reutilizar a Atividade 1 mas tente ser criativo melhorando o projeto anterior.
2. Configurar dashboard com widgets para o device criado - DICA: Como o Dashboard tem limite de
   10 Variaveis, pode usar variaveis de contexto para aumentar esse limite. Veja Aulas da Semana 6
3. No Wokwi, publicar telemetria MQTT em algum Broker (MQTTx, HiveMQ, Mosquito, etc)
4. No Ubidots, configurar integração MQTT usando Token do device/variáveis para consumir dados do
   Broker (via bridge, plugin, ou fluxo suportado pela plataforma, pode ser criado um outro sistema
   com ESP32 como retransmissor como ex. da Semana 7).
5. Validar se os dados do device estão atualizando no dashboard do Ubidots - Gere um relatorio
   (pode ser capturado do monitor serial) da origem dos dados e comparar com o resultado final.
   Acrescente na documentação.
6. Inserir print do dashboard atualizado na documentação
7. Criar uma documentação adequada mostrando um rascunho simples da arquitetura, objetivo do
   projeto, descrição geral do sistema, explicação de cada componente, seu fluxo de interação entre
   eles, descrição das variaveis de publicação e subscrição e conclusoes. Faça um documento formal
   com capa, titulo, indices, etc.
8. Fazer um pequeno video de poucos minutos mostrando o funcionamento dos sistemas, pode usar
   captura de tela.

### Elaboração do sistema

O enunciado pede para elaborar um sistema representado pelo diagrama abaixo, que também mostra onde
o protocolo MQTT precisa ser usado.

| ![Diagrama em blocos descrito no enunciado](Block_diagram_0.png) |
|:--:| 
| _Diagrama em blocos descrito no enunciado_ |

As linhas tracejadas representam a conexão e o bloco que foi deixado em aberto, a critério do aluno.

É fácil entender o interesse didático do sistema proposto, mas para que esse sistema tenha coerência
em uma aplicação real, imaginei um sistema em que o broker mais à esquerda possui dados que a
plataforma Ubidots não possui e a plataforma Ubidots solicita dados ao broker. Para ser mais
coerente, é preciso que o Ubidots não solicite exatamente os mesmos dados que o broker já possui,
mas sim, uma versão processada dos dados. Caso contrário, após algumas requisições, o Ubidots teria
o mesmo conjunto de dados que o broker e o broker se tornaria dispensável.

Essa abordagem não limita a tecnologia de integração MQTT permanece flexível. Escolhi Node-RED por
interesse didáticos.

### Sistema elaborado: PoC - Serviço de Temperatura por Localização

Podemos imaginar este sistema como um serviço em que um cliente solicita a temperatura ambiente em
uma localização escolhida. A temperatura que o cliente recebe é inferida a partir de uma tabela com
algumas posições e a temperatura instantânea nessas posições, de forma que quando o cliente solicita
a temperatura em um ponto, o serviço usa fórmulas matemáticas para inferir a posição nesse ponto e
informa o cliente, além de também incrementar o acumulador de quanto aquelas requisições vão custar
ao cliente.

A figura abaixo apresenta o diagrama em blocos do sistema. Em comparação com diagrama em blocos
anterior, o diagrama abaixo define mostra três ESP32 do lado como entrada de dados de sensores,
define que o broker é a plataforma MQTTx, que a integração MQTT é feita pela plataforma Node-RED,
define o sentido do fluxo de dados e mostra que o protocolo entre o broker a o Node-RED é o MQTT.

| ![Diagrama em blocos do Serviço de Temperatura por Localização](Block_diagram_1.png) |
|:--:| 
| _Diagrama em blocos do Serviço de Temperatura por Localização_ |

O sistema, tal qual está, é apenas uma PoC (_Proof of Concept_ - Prova de Conceito), pois a tabela
suporta apenas 3 posições e a inferência é feita usando a fórmula de um plano de álgebra linear.
Contudo, com a substituição da fórmula, o sistema poderia ser expandido para usar mais pontos e
também para informar outros dados, como umidade do ar, índice de poluição, ou pressão atmosférica.
O custo para o cliente também está bastante simplificado, apenas incrementando em $0,01 a cada
requisição.

Apesar dessas limitações, o sistema é válido como prova de conceito e também implementa o sistema
requisitado pela atividade do curso.

As próximas seções detalham a implementação, apresentam como executar o sistema, os resultados, e os
pontos de melhoria que poderia haver com mais tempo. O relatório fecha com uma breve conclusão.

## Implementação

### Visão geral
Este sistema pode ser dividido em três partes:

1. Coleta de dados: esta parte, que contém os sensores microprocessados e o broker MQTTx, coleta
   dados e os envia para o servidor MQTTx.
2. Interface com o cliente: esta parte, que inclui um _dashboard_ no Ubidots e o broker do próprio
   Ubidots, tem a interface para o cliente fazer a requisição, apresenta em um mapa a temperatura
   informada pelo sistema e também apresenta o custo para o cliente.
3. Aplicação Node-RED: é o coração do sistema, ela armazena os dados dos sensores, identifica quando
   o cliente faz a requisição, calcula a temperatura na localização escolhida, e faz a conta de
   quanto o cliente deve pagar.

A seguir, este documento apresento cada uma dessas partes e termina com uma
seção descrevendo os tópicos MQTT utilizados.

### Coleta de dados

Neste sistema, três placas sensoras enviam dados de temperatura ambiente e localização para o broker
MQTTx. Foi determinado que haveria apenas três placas pois o método de inferência de temperatura se
tornaria muito mais complexo com quatro ou mais placas.

Utilizei o microcontrolador ESP32 simulado pelo site Wokwi.com, como usado constantemente no curso.
Para medir a temperatura, usei um sensor do tipo NTC. Para a localização, como não havia um
componente GPS disponível no Wokwi.com, defini a posição manualmente, para cada um dos sensores, da
seguinte forma:

| **Identificação** | **Latitude** | **Longitude** | **Descrição** |
| ----------------- | ------------ | ------------- | ------------- |
| Sensor 0          | -22.8162268  | -47.0451902   | Localização do CPQD |
| Sensor 1          | -22.9027350  | -47.0563132   | Localização da FIAP |
| Sensor 2          | -22.8517996  | -47.1284529   | Localização do [CTI](https://www.gov.br/cti/pt-br) |

Inseri um DIP-switch para fazer a seleção dos sensores, como pode ser visto na
figura abaixo.

| ![ESP32 usado como sensor de temperatura](ESP32_temperature_sensor.png) |
|:--:| 
| _ESP32 usado como sensor de temperatura_ |

Como descrito na figura, as chaves 7 e 8 configuram a identificação do sensor. Essa configuração só
é aceita durante a inicialização do dispositivo, uma vez que não faz sentido uma atualização
dinâmica da identificação.

O ESP32 coleta os dados de temperatura e envia para o broker MQTTx a cada 2 segundos.

### Interface com o cliente

Como especificado no enunciado do exercício, criei um _dashboard_ com a interface do cliente. Para
solicitar a temperatura, é preciso selecionar a posição —latitude e longitude— e
então enviar a solicitação ao Node-RED.

Para fazer a requisição, criei o dispositivo _"Temperature Requester"_ e adicionei nele três
variáveis manualmente:

* req_lat - latitude da requisição, numérico;
* req_lon - longitude da requisição, numérico;
* req_run - gatilho da requisição, numérico, mas seu valor não importa, desde que ele mude.

Idealmente, seria possível enviar tanto a localização como a requisição em uma única mensagem MQTT.
Contudo, o Ubidots não consegue não consegue agregar os dados no MQTT e, por isso ele envia uma
mensagem para a latitude, uma para a longitude e outra para o gatilho.

Ao receber a resposta via MQTT, o Ubidots cria automaticamente duas variáveis nesse mesmo
dispositivo, baseado no tópico MQTT que chega:

* cost - variável numérica com o custo acumulado do serviço
* position - apesar do nome, esta variável contém tanto a posição como a temperatura ambiente

A figura abaixo mostra o dispositivo _"Temperature Requester"_, já com as 5 variáveis descritas
acima.

| ![Dispositivo "Temperature Requester" no Ubidots](Ubidots_device.png) |
|:--:| 
| _Dispositivo "Temperature Requester" no Ubidots_ |

Já a figura abaixo mostra o _dashboard_ do Ubidots para fazer a requisição, já com algumas
respostas:

| ![Dashboard no Ubidots](Ubidots_dashboard.png) |
|:--:| 
| _Dashboard no Ubidots_ |

Para aumentar a usabilidade do dashboard, fiz algumas customizações dignas de nota:

#### _Sliders_ de latitude e longitude
Na falta de alternativa melhor, o sistema utiliza _sliders_ para entrar com a latitude e longitude.
Porém, como pode-se ver na seção anterior, a variação entre os valores máximo e mínimo, tanto de
latitude, como de longitude, é pequena:

* latitude:
  * máxima: -22.8162268°
  * mínima: -22.9027350°
* longitude:
  * máxima: -47.0451902°
  * mínima: -47.1284529°

Assim, os valores padrão dos sliders, de 0 a 100 não são adequados. Limitei esses valores à faixa
entre -22.81° e -22.91° para latitude e à faixa entre -47.04° e -47.13° para longitude, ambas com
passo de 0.001°. Além disso, coloquei o slider de latitude na vertical e o slider de longitude na
horizontal, para uniformizar com a visão de mapas mais comum, onde o norte está em cima.

#### Gatilho de requisição
O Ubidots disponibiliza um controle chamado _switch_, que permite alternar uma variável entre os
valores 0 e 1. Esse controle muda a cor e o texto apresentado conforme o valor selecionado. Contudo,
neste caso, o valor não é importante, mas sim a mudança e, por isso, editei o controle de forma que
ele tivesse a mesma cor, independente do seu valor, e suprimi o texto apresentado.

#### Mapa
Este foi o elemento mais difícil de customizar no _dashboard_, sendo necessário ler toda a
documentação dele, presente em
<https://help.ubidots.com/en/articles/1712418-create-map-widgets-in-ubidots>. 

O que tornou seu uso particularmente difícil foi o fato dele não usar uma hierarquia de variáveis
simples, mas sim uma que classifica a latitude e a longitude como propriedades dentro de um contexto
dentro de uma variável que contém o dado de temperatura.

Mas, quando consegui formatar a variável conforme o modelo esperado, o mapa passou a funcionar como
esperado. O formato da variável MQTT será detalhada em uma das próximas seções.

Dado que o mapa é o elemento mais chamativo do dashboard, achei interessante adicionar algumas
referências nele para deixá-lo mais intuitivo de compreender. Por isso, criei dois outros
dispositivos: CPQD e FIAP, conforme pode ser visto na figura abaixo:

| ![Lista de dispositivos no Ubidots](Ubidots_device_list.png) |
|:--:| 
| _Lista de dispositivos no Ubidots_ |

O único objetivo deles é inserir referências visuais no mapa. Configurei suas posições conforme a
posição real (descrita na tabela da seção anterior) e inseri-os no mapa, como pode ser visto na
figura abaixo, em que a antena vermelha assinala a posição do CPQD e o prédio vermelho, supostamente
uma escola, assinala a posição da FIAP. Não foi possível inserir mais marcadores pois a versão
gratuita do Ubidots não permite criar mais do que 3 dispositivos.

| ![Marcadores no mapa do Ubidots](Ubidots_map_detail.png) |
|:--:| 
| _Marcadores no mapa do Ubidots_ |

#### Demais elementos do dashboard
Um indicador de temperatura e um texto com o valor a ser pago completam o _dashboard_. Apesar de
serem elementos importantes, sua criação e funcionamento são de entendimento trivial.

### Aplicação Node-RED

O Node-RED fica monitorando o broker e assim que esses dados chegam, o Node-RED os armazena em
variáveis internas.
O Node-RED também fica monitorando os comandos 


## Como executar o sistema

#### Programa node red


Se você já tiver docker instalado:

    docker run -it -p 1880:1880 -v node_red_data:/data --name mynodered nodered/node-red

## Resultados

## Conclusão
