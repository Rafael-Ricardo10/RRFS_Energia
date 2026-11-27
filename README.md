# ☀️ RRFS Energia

**Sistema de gerenciamento de energia solar offgrid para ESP32.**

Feito para tirar a máxima eficiência da energia gerada em uma casa **100% offgrid** (sem rede da concessionária). Já foi testado em ambiente real.

![versão](https://img.shields.io/badge/versão-1.0.1-blue)
![plataforma](https://img.shields.io/badge/plataforma-ESP32-green)
![licença](https://img.shields.io/badge/open%20source-sim-brightgreen)

---

## 📑 Índice

- [A ideia principal](#-a-ideia-principal)
- [Funcionalidades](#-funcionalidades)
- [Como funciona](#️-como-funciona)
- [Hardware necessário](#-hardware-necessário)
- [Pinagem do ESP32](#-pinagem-do-esp32)
- [Circuitos de sinal](#-circuitos-de-sinal)
- [Instalação do firmware](#-instalação-do-firmware)
- [Primeiro acesso e assistente](#-primeiro-acesso-e-assistente-de-configuração)
- [Páginas do sistema](#️-páginas-do-sistema)
- [Proteção da bateria](#-proteção-da-bateria)
- [Planejador de energia](#-planejador-de-energia)
- [Aproveitamento do excedente solar](#️-aproveitamento-do-excedente-solar)
- [Aprendizado do consumo](#-aprendizado-do-consumo-padrões)
- [Iluminação externa](#-iluminação-externa)
- [Saúde da bateria](#️-saúde-da-bateria)
- [Previsão solar](#️-previsão-de-geração-solar)
- [Wi-Fi e modo sem internet](#-wi-fi-e-modo-sem-internet)
- [Atualização OTA](#-atualização-de-firmware-ota)
- [Solução de problemas](#-solução-de-problemas)
- [Limitações conhecidas](#️-limitações-conhecidas)
- [Em desenvolvimento](#-em-desenvolvimento)
- [Aviso de segurança](#️-aviso-de-segurança)
- [Apoie o projeto](#️-apoie-o-projeto)
- [Contribuindo](#-contribuindo)

---

## 💡 A ideia principal

A maior parte dos equipamentos de uma casa que ficam ligados o tempo todo consome pouca energia. Os de alto consumo são usados só de vez em quando. Um inversor grande ligado 24 h gasta muita energia da bateria **só para ficar ligado**, mesmo sem carga.

O RRFS Energia usa **dois inversores**:

| Inversor | Potência (exemplo) | Equipamentos | Funcionamento |
|---|---|---|---|
| 🟢 **Baixa potência** | ~500 W | Carregador de celular, notebook, servidor, roteador, lâmpadas | Fica ligado a maior parte do tempo |
| 🔴 **Alta potência** | ~5000 W | Geladeira, forno, máquina de lavar, panela elétrica | Fica desligado e **só liga quando necessário** |

Assim, o inversor grande não desperdiça energia da bateria.

---

## ✨ Funcionalidades

- 🔋 **Reserva de energia configurável:** você define quanto da bateria quer manter de reserva.
- 🛡️ **Proteção por SOC:** desliga inversores e iluminação ao chegar na reserva e só religa com folga (histerese). O painel mostra em que SOC o sistema volta a ligar.
- 📅 **Planejamento de energia:** simula as próximas horas (consumo × geração prevista) e decide o que desligar antes de faltar energia.
- 🌳 **Controle de iluminação externa:** dois sistemas (claridade e jardim), com medição automática do consumo de cada um.
- 🌤️ **Previsão de geração solar:** usa GPS, configuração dos painéis e do controlador, e compara **8 modelos meteorológicos**, dando mais peso aos que mais acertam.
- ⚠️ **Ação preventiva:** se prevê falta de energia, desliga a iluminação externa, avisa e recomenda economizar.
- 🧠 **Aprendizado do consumo:** analisa a corrente, cria padrões de uso (forno, compressor, máquina de lavar) e planeja com base neles.
- ☀️ **Aproveitamento do excedente:** com a bateria cheia e sol sobrando, liga o inversor de alta sozinho.
- ❤️‍🩹 **Saúde da bateria:** mede resistência interna e capacidade ao longo do tempo.
- 🔌 **Diagnóstico e religamento automático** dos inversores.
- 📈 **Histórico de 30 dias** de geração e consumo, hora a hora.
- 🖥️ **Interface web** acessível pelo navegador, no celular ou computador.
- 🚀 **Atualização de firmware pelo navegador (OTA).**

---

## ⚙️ Como funciona

Um ESP32 controla todo o sistema.

### Acionamento do inversor de alta potência

- **Equipamentos manuais** (máquina de lavar, forno etc.): um interruptor em uma tomada separada envia um sinal de 3,3 V ao pino de **pedido da casa**. Ao ligar o interruptor, o sistema liga o inversor de alta. Ao desligar, ele desliga.
- **Geladeira (automático):** o sinal é ligado ao termostato. Quando o compressor liga, o sistema liga o inversor de alta automaticamente.
- **Repouso do compressor:** depois de desligar o inversor de alta, o sistema espera **3 minutos** antes de religar, para proteger o compressor da geladeira.

### Monitoramento dos inversores

O ESP32 recebe o sinal de status de cada inversor (ligado/desligado), com a tensão reduzida a um nível seguro. Com isso o sistema:

- sabe quando cada inversor está energizado;
- faz diagnósticos;
- religa automaticamente um inversor que falhou.

### Comunicação com a bateria (Bluetooth)

O sistema lê a BMS da bateria via **Bluetooth (BLE)**: tensão, corrente, SOC, capacidade e ciclos.

> 📌 Esta versão exige uma bateria com **BMS Bluetooth** do tipo **JBD / LanmaoBMS** (serviço `FFFA`). ⚠️ CONFIRMAR modelos testados.
>
> Se a sua bateria não tem Bluetooth, veja a pasta **Desenvolvimento**, com versões antigas que usam sensores e outros métodos de leitura. Elas são de desenvolvimento e podem ter bugs. Use como base ou referência.

### Corrente da casa

O sistema mede a corrente do painel por um sensor no ESP32 e a corrente da bateria pela BMS. A corrente da casa é calculada:

```
Casa = Painel − Bateria
```

---

## 🧰 Hardware necessário

| Item | Observação |
|---|---|
| ESP32 (DevKit de 30/38 pinos) | ⚠️ CONFIRMAR placa usada |
| Bateria com BMS Bluetooth JBD/LanmaoBMS | Banco único, série, paralelo ou série+paralelo |
| 2 inversores | Baixa e alta potência |
| Módulo de relé de 4 canais | Inv. Baixa, Inv. Alta, Claridade, Jardim |
| Sensor de corrente do painel | Saída analógica, ~35,9 mV/A, offset 2,5 V (calibrável) |
| Divisores de tensão | Para os 3 sinais (AC Baixa, AC Alta, Pedido da Casa) |
| Controlador de carga solar | Informar a corrente máxima em A na configuração |
| Fonte 5 V para o ESP32 e relés | Estável |
| Fusíveis, disjuntores, DPS | Obrigatórios |

> O sensor do painel usa o ADC do ESP32, que trabalha de 0 a 3,3 V. Se o seu sensor tem saída de até 5 V, **use divisor de tensão**.

---

## 📌 Pinagem do ESP32

| GPIO | Função | Tipo |
|---|---|---|
| 2 | LED azul da placa (status) | Saída |
| 0 | Botão BOOT (segurar 10 s = reset de fábrica) | Entrada |
| 34 | Sensor de corrente do painel | Entrada analógica |
| 35 | Sinal AC do **Inversor Baixa** (diagnóstico) | Entrada analógica |
| 36 | Sinal AC do **Inversor Alta** (diagnóstico) | Entrada analógica |
| 39 | **Pedido da casa** (liga o Inv. Alta) | Entrada analógica |
| 26 | Relé do **Inversor Baixa** | Saída |
| 25 | Relé do **Inversor Alta** | Saída |
| 33 | Relé da **iluminação 1 (claridade)** | Saída |
| 32 | Relé da **iluminação 2 (jardim)** | Saída |

Cada relé pode ser **invertido** pela página *Controle*, para módulos ativos em LOW ou HIGH.

---

## 🔌 Circuitos de sinal

Os pinos 35, 36 e 39 leem a presença de tensão em corrente alternada ou contínua, por divisor de tensão.

- **Limiar padrão:** 400 mV, ajustável por sinal na página *Controle*.
- **Histerese:** liga acima de limiar + 50 mV, desliga abaixo de limiar − 40 mV.
- **Confirmação por tempo** (evita falso disparo):

| Sinal | Confirma ao ligar | Confirma ao desligar |
|---|---|---|
| AC Inversor Baixa (35) | 0,4 s | 0,4 s |
| AC Inversor Alta (36) | 0,1 s | 5 s |
| Pedido da Casa (39) | 5 s | 10 s |

> ⚠️ **Nunca ligue 127/220 V direto no ESP32.** Use divisor, transformador de sinal ou optoacoplador, com isolação adequada.

---

## 📥 Instalação do firmware

### Requisitos

- **Arduino IDE** com suporte a ESP32 (core Arduino-ESP32 3.x).
- Biblioteca **NimBLE-Arduino 2.x**.
- Bibliotecas incluídas no core: `WiFi`, `WebServer`, `Preferences`, `LittleFS`, `HTTPClient`, `Update`, `DNSServer`.

### Passos

1. Instale o suporte a ESP32 e a biblioteca **NimBLE-Arduino** pelo Gerenciador de Bibliotecas.
2. Abra o arquivo `.ino` do projeto.
3. Selecione a placa **ESP32 Dev Module**.
4. Escolha um **esquema de partição com área SPIFFS/LittleFS** (por exemplo, *Default 4MB with spiffs*). Sem isso, os padrões de consumo ficam só na RAM e se perdem ao reiniciar.
5. Compile e grave.

> 💡 Para atualizações futuras use o **OTA**, pelo navegador, sem cabo.

---

## 🚀 Primeiro acesso e assistente de configuração

Depois de gravar, o ESP32 cria uma rede Wi-Fi aberta chamada:

```
RRFS Energia
```

1. Conecte o celular ou computador nessa rede.
2. Abra o navegador em **http://10.0.0.1**
3. Crie seu usuário e senha.
4. Siga o **assistente de 4 passos**:

| Passo | O que fazer |
|---|---|
| 1. BMS | Toque em *Buscar BMS* e escolha a sua na lista (marcada com ★ se parecer JBD). Feche antes o app da BMS no celular, pois ele trava a conexão. |
| 2. Banco | Informe como as baterias estão ligadas: único, paralelo, série ou série+paralelo. |
| 3. Calibração | Calibre o sensor do painel (veja abaixo). |
| 4. Rede | Escolha o Wi-Fi do roteador e defina **IP fixo**, ou use só o ponto de acesso. |

Você pode **pular** os passos 1 a 3 e configurar depois.

### Calibração do sensor do painel

1. Deixe **só o controlador de carga e a bateria** ligados (inversores e cargas desligados).
2. Com sol no painel, toda a corrente vai para a bateria.
3. Aperte **Calibrar Agora**. O sistema usa a corrente da bateria (lida pela BMS) como referência exata.

### Depois do assistente

Na página **Previsão**, informe latitude, longitude, potência dos painéis (kW) e a corrente do controlador de carga (A).
Na página **Bateria**, defina a **reserva de proteção**.

---

## 🖥️ Páginas do sistema

| Página | Ícone | O que mostra |
|---|---|---|
| Painel | 📊 | Resumo ao vivo: banco, autonomia, iluminação, saúde, planejador, inversores, excedente, sinais, previsão e fluxo de energia |
| Bateria | 🔋 | Configuração do banco, reserva de proteção, dados brutos da BMS |
| Controle | ⚙️ | Limiares, iluminação, inversão de relés, **modo teste** dos relés |
| Sensores | 🔍 | Leituras ao vivo e calibração do painel |
| Previsão | ☀️ | Configuração, precisão, pesos dos 8 modelos, previsão de 7 dias |
| Histórico | 📈 | Totais, padrões aprendidos, eventos do dia, 30 dias, gráfico em tempo real |
| Config | ⚙️ | Data/hora, senha, apagar histórico, logs, BMS, Wi-Fi, OTA, reiniciar |
| Saúde | 🩺 | Resistência interna, capacidade, tendência |

Outras rotas: `/bms` (busca e conexão da BMS), `/logs-salvos` (últimos 100 logs), `/logs-txt` (download dos logs), `/ota`.

---

## 🛡️ Proteção da bateria

Você define uma **reserva** (padrão 15%, de 0 a 50%).

| Evento | SOC |
|---|---|
| Proteção **liga** | SOC ≤ reserva |
| Proteção **desliga** | SOC ≥ reserva + **5%** (histerese) |

Enquanto ativa, o sistema:

- desliga o **Inversor Baixa**, o **Inversor Alta** e as duas **iluminações**;
- **verifica o nível real dos pinos** dos relés a cada 2 s e reafirma o OFF, mesmo que a flag interna esteja errada (`garantirRelesOffProtecao()`);
- mostra no painel o **SOC em que vai religar**, quanto falta (% e Ah) e uma **previsão de tempo**, calculada pela corrente de carga atual.

A proteção **sobrevive a reinício**: se o ESP reiniciar com a proteção ativa, ela continua ativa até o BMS informar um SOC seguro. Se o BMS ficar 30 min sem dados, ela é liberada.

### Quando cada item volta (reserva 15%, limiar da iluminação 20%)

| Item | Volta com SOC de | Condições |
|---|---|---|
| Inversor Baixa | 20% | Liga em até 0,5 s |
| Inversor Alta | 20% | Casa pedindo + repouso de 3 min |
| Claridade | 25% | Só à noite, sem sol |
| Jardim | 40% | Só à noite, sem sol, dentro do horário |

> 💡 Para bancos pequenos, considere aumentar `PROT_HISTERESE_PCT` (ex.: 10) para evitar ciclos de liga/desliga em dias nublados.

---

## 🧠 Planejador de energia

O planejador simula as próximas horas e decide o que cortar **antes** de a bateria chegar na reserva.

```
consumo(h) = base(h) + eventos esperados(h) + claridade(h) + jardim(h)
bateria(h+1) = bateria(h) + geração prevista(h) − consumo(h)
```

**Ordem de corte:**

1. 🌿 Jardim
2. 💡 Claridade
3. ⚠️ Economizar (o painel mostra **quantos A/W reduzir** e dicas)

- O horizonte é dinâmico: vai até o **fim do sol do dia seguinte**, no máximo 36 h.
- Usa **confirmação por tempo**: 5 min para bloquear, 10 min para liberar. Não pisca.
- Só começa a agir após **24 h de dados** de consumo aprendidos. Antes disso, não desliga a iluminação.
- Exige: previsão solar ativa, BMS com dados, NTP sincronizado e consumo medido.

---

## ☀️ Aproveitamento do excedente solar

Quando a bateria está cheia e os painéis ainda geram muita energia, o sistema liga o **inversor de alta** para você usar equipamentos pesados sem tirar da bateria.

**Entra em excesso quando, por 60 s:**

- SOC ≥ 99%;
- painel gerando ≥ 0,8 A;
- bateria sem descarregar;
- previsão mostra sobra;
- não é o fim da geração do dia;
- inversor de alta em repouso há mais de 3 min.

**Sai quando:**

- o SOC cai 1% e a bateria não está carregando;
- o painel para de gerar;
- o SOC fica abaixo de 90%;
- o inversor de alta não energiza;
- a proteção da bateria ou a falta de dados do BMS ocorre.

Se o compressor da geladeira estiver ligado, ela gela o máximo possível durante o dia e o compressor liga menos à noite, economizando bateria.

Há pausa entre excessos (5 min, ou 15 min após uma falha) para evitar liga/desliga.

---

## 🧩 Aprendizado do consumo (padrões)

O sistema separa o consumo da casa em duas partes:

- **Base estável:** cargas pequenas e constantes, ligadas no inversor de baixa (luz, roteador, notebook).
- **Eventos:** cargas pesadas e temporárias, ligadas no inversor de alta por pedido da casa (forno, compressor, máquina de lavar).

O detector mede, para cada evento: horário, duração, corrente média e pico acima da base, e quanto do tempo o painel cobria a casa.

Os eventos parecidos (corrente ±0,5 A ou ±20%, duração em até ~2×) são agrupados em **padrões** (até 8). Para cada padrão o sistema calcula quantas vezes por dia ocorre e em quais horas, dando **mais peso aos dias recentes**. O planejador converte isso em Ah esperados por hora.

- **Armazenamento:** 30 dias em LittleFS (`/pad30.bin`), com CRC e gravação segura (arquivo temporário + rename).
- **Pausa no excedente:** durante a sobra solar, nada é aprendido.
- **Pausa na medição da iluminação:** o sistema não aprende enquanto liga e desliga lâmpadas para medir.
- A página **Histórico** mostra o estado do detector em tempo real: *corrente estável*, *subindo*, *padrão conhecido em andamento* ou *corrente nova (aprendendo)*.

---

## 🌳 Iluminação externa

Dois sistemas independentes, que você ativa na página *Controle*:

| Sistema | Relé | Quando liga | Prioridade |
|---|---|---|---|
| 💡 **Claridade** | GPIO33 | À noite, das 19:00 às 05:30 | Última a ser desligada |
| 🌿 **Jardim** | GPIO32 | A partir das 19:00 até a hora que você escolher (padrão 23h) | **Primeira** a ser desligada |

**Só liga quando:** é noite, o painel não está gerando, o Inversor Baixa está ligado, o SOC está acima do limiar e o planejador permite.

**Medição automática do consumo:**
1. Espera a corrente da casa ficar estável.
2. Liga a lâmpada, espera 2,5 s e mede por 6 s. O consumo é a diferença em amperes.
3. Restaura o estado e mede a próxima.

Repete a cada **7 dias**, ao ativar uma lâmpada nova, por pedido manual (*Medir consumo agora*), ou quando o detector perceber que o consumo mudou. A página guarda o **uso dos últimos 30 dias** (minutos ligada e Wh).

---

## ❤️‍🩹 Saúde da bateria

| Indicador | Como mede |
|---|---|
| ⚡ **Resistência interna** (principal) | Degrau de corrente ≥ 8 A entre dois quadros consecutivos da BMS: R = ΔV / ΔI. Mediana das últimas 8 medidas, comparada com a referência (8 primeiras). |
| 🔋 **Capacidade** | Integra a corrente entre transições de SOC, em janelas de pelo menos 20% (SOC de 10% a 96%). |

**Classificação da resistência (vs. referência):** Normal ≤ 1,15× · Levemente elevada ≤ 1,30× · Elevada ≤ 1,60× · Alta acima disso.

> ℹ️ O SOC da BMS é um contador de Ah. A "capacidade" medida mostra a **coerência entre BMS e sensor**, e **não prova** que as células estão novas. Use sempre a tendência, não um valor isolado.

---

## 🌤️ Previsão de geração solar

- Usa a API gratuita **Open-Meteo** (sem chave).
- Compara **8 modelos**: ECMWF, GFS, ICON, GEM, Météo-France, JMA, UKMO e CMA.
- O peso de cada modelo é proporcional a `1 / erro²`: quem acerta mais pesa mais.
- A previsão é **calibrada** com a geração real. O painel deve gerar igual ou acima do previsto.
- **Horas censuradas:** quando a bateria está cheia (SOC ≥ 92%) ou há excedente ativo, o painel só entrega o que a casa consome. Essas horas **não punem** modelo nem fator.
- Considera o **limite do controlador de carga** (A × 95%).
- Mostra a **precisão** (taxa de acerto) e a previsão de **7 dias**.
- Se a memória estiver baixa, cai para o modo de modelo único (`best_match`).

---

## 📶 Wi-Fi e modo sem internet

| Situação | Funcionamento |
|---|---|
| 🌐 Com roteador e internet | Funcionalidades completas |
| 📴 Sem roteador | Funciona normalmente. Conecte na rede **RRFS Energia** e acesse **10.0.0.1** |
| 📴 Sem internet | Funciona com limitações (veja abaixo) |

**Recuperação automática:** se o roteador cair, o ESP sobe sozinho o ponto de acesso `RRFS Energia` (10.0.0.1) e continua tentando voltar ao roteador a cada minuto. Ele também detecta "Wi-Fi associado mas sem tráfego" por ping no gateway e reinicia o rádio. Após 30 min nesse estado, reinicia o ESP.

### ⚠️ Limitações sem internet

Sem internet o sistema não consegue atualizar data e hora, nem obter a previsão solar, e o **planejador não age** (ele depende da previsão).

Mesmo assim continua funcionando: leitura da bateria, **proteção por SOC**, controle e diagnóstico dos inversores, histórico de geração e consumo, aprendizado de padrões (se o relógio já estava sincronizado) e a interface local.

> 💡 O relógio precisa estar sincronizado por NTP para o histórico, os padrões e o excedente. Você também pode ajustar a hora manualmente.

---

## 🚀 Atualização de firmware (OTA)

1. No Arduino IDE: **Sketch → Exportar binário compilado**.
2. No navegador, abra **Config → OTA**.
3. Envie o arquivo **`.ino.bin`**. **Não use** `bootloader`, `merged` ou `partitions`.
4. O Bluetooth é desligado durante a atualização. Feche outras abas e aguarde o reinício.

Os padrões de consumo são salvos antes de reiniciar.

---

## 🔧 Solução de problemas

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| BMS não aparece na busca | App da BMS no celular está conectado | Feche o app e busque de novo |
| "BMS sem dados" | Distância ou interferência | Aproxime o ESP32 da bateria |
| Inversor continua ligado com a proteção ativa e nenhum log `forcado OFF` | Problema físico: relé travado, fiação em bypass ou inversão errada | Use o **Modo Teste** em *Controle*, veja se o relé estala e se o AC muda |
| Sistema liga sozinho depois da proteção | Comportamento normal | Religa quando o SOC chega em reserva + 5% |
| Liga e desliga em ciclo em dia nublado | Histerese pequena | Aumente `PROT_HISTERESE_PCT` ou a reserva |
| Planejador mostra "Aguardando dados" | Falta previsão, BMS, NTP ou 24 h de aprendizado | Veja o motivo exibido no card |
| Padrões somem ao reiniciar | Sem partição LittleFS | Escolha um esquema de partição com SPIFFS |
| Corrente do painel errada | Sensor descalibrado | Recalibre na página *Sensores* |

**Logs:** *Config → Logs* mostra os últimos 100 eventos (`/logs-txt` baixa em arquivo). Registros importantes: `PROTECAO ON/OFF`, `Prot: ... forcado OFF`, `Excesso ON`, `Plano: ...`.

**Reset de fábrica:** segure o botão **BOOT** por 10 s.

---

## ⚠️ Limitações conhecidas

- Exige **BMS Bluetooth JBD/LanmaoBMS**.
- A leitura da BMS atualiza a cada cerca de 1 s.
- O firmware só garante que **comanda** o relé. Ele não detecta falha física (contatos colados, bypass).
- O planejador precisa de internet para a previsão solar.
- O BMS não informa temperatura neste firmware, então a resistência interna varia com a temperatura.
- Gerador ainda não suportado.

---

## 🚧 Em desenvolvimento

**Gerador:** integração de um gerador adaptado, com saída na tensão DC do controlador de carga. O sistema vai controlá-lo e ligá-lo quando necessário. Será lançado em uma versão futura.

---

## ⚠️ Aviso de segurança

Este projeto envolve **alta tensão, corrente elevada e baterias**, que podem causar choque, incêndio e danos aos equipamentos.

- Os pinos do ESP32 trabalham com **3,3 V**. Tensões maiores podem danificá-lo. Use divisor de tensão, optoacoplador ou outro circuito de isolamento.
- **Nunca** ligue a rede elétrica (127/220 V) direto ao ESP32.
- Instale proteções (fusíveis, disjuntores, DPS) e siga as normas elétricas.
- **Desligue a bateria antes de mexer na fiação.**
- Se não tiver experiência, procure um profissional.

**Use por sua conta e risco.** O autor não se responsabiliza por danos causados pelo uso ou pela montagem incorreta.

---

## ❤️ Apoie o projeto

O RRFS Energia é um projeto **open source**. Doações são muito bem-vindas e me incentivam a continuar desenvolvendo.

| Forma | Chave |
|---|---|
| **PIX** | `0cd822ec-e5a8-48cb-b17f-a9c4233c7b59` |
| **Bitcoin (BTC)** | `bc1qqz3l07skhpvh4k5dzhcgyh342c76f2y4vmka27` |

**Obrigado pelo apoio!** 🙌

---

## 🤝 Contribuindo

Sugestões, relatos de bugs e contribuições são bem-vindos. Abra uma **Issue** ou envie um **Pull Request**.

Ao relatar um bug, ajude com: versão do firmware, modelo da BMS, o texto exibido no painel e os logs (*Config → Logs*).
