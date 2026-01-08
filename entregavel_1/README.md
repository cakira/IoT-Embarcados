# Entregável 1

## Enunciado

### Atividade 1 – Definição de Cenário, Sensores/Atuadores e Tópicos MQTT

**Ferramentas:** Wokwi (ou equivalente) + Cliente MQTT + Broker MQTT (Mosquitto,
                 HiveMQ ou MQTTX).

**Itens a serem desenvolvidos:**
* Escolher um tema/cenário IoT (livre pelo estudante).
* Definir no mínimo 3 sensores e 1 atuador, com unidade e faixa.
* Criar hierarquia de tópicos MQTT coerente.
* Publicar 1 mensagem de teste e assinar o tópico no cliente MQTT.

**Entregáveis:** link de um vídeo mostrando todos os itens implementados, além
                 do link do projeto desenvolvido no Wokwi (ou plataforma
                equivalente).

## Atividade

### Controlador de nível de água em uma banheira

#### Sensores
* Temperatura da água 1
* Temperatura da água 2
* Nível (analógico) de água
* Botão do tipo liga/desliga - para poder desativar o sistema

#### Atuadores
* Controle de água quente (digital, do tipo aberto/fechado)
* Controle de água fria (digital, do tipo aberto/fechado)
* Dreno de água (digital, do tipo aberto/fechado) - permite o escoamento de água

#### Atuação via Internet
* Apresentação da temperatura média da água
* Controle da temperatura máxima e mínima aceitável
* Controle do nível máximo e mínimo aceitável de água

#### Funcionamento
O sistema monitora continuamente a temperatura nos dois sensores de temperatura
da água e envia a média das duas leituras para o broker MQTT. Ele também lê o
nível da água e envia para o broker MQTT.

No broker MQTT o usuário pode controlar a temperatura desejada e o nível de
água, ambos com uma definição de valor máximo e mínimo aceitável.

Quando o botão liga/desliga estiver no modo "liga", o sistema controlará o nível
e temperatura da água. A lógica ainda será definida.
