# LMAC-1 no FLoRa

Simulador usado na minha monografia de graduação em Ciência da Computação (UENF, 2026), que compara o protocolo LMAC-1 (Gamage et al., 2023) com o ALOHA do LoRaWAN, usando a taxa de perda de pacotes (PLR) como métrica.

O código parte do [FLoRa](https://github.com/florasim/flora) 1.1.0 (tag `v1.1.0`, commit `51018f8`), com o histórico original mantido. As alterações estão nos commits seguintes e são resumidas abaixo. Mais informações sobre o FLoRa em [flora.aalto.fi](http://flora.aalto.fi/).

## Versões

- OMNeT++ 6.3.0
- INET 4.4.2, sem alterações (não incluído neste repositório)
- FLoRa 1.1.0, com as alterações deste repositório

## Instalação

O FLoRa procura o INET numa pasta irmã chamada `inet4.4`, e a IDE espera que o projeto se chame `flora`. Por isso, clone este repositório com o nome `flora`, na mesma pasta do INET:

```
git clone https://github.com/Hmdre/lmac1-flora.git flora
```

### Pela IDE do OMNeT++

1. Instale o INET 4.4.2 numa pasta `inet4.4` do workspace e compile.
2. Importe o projeto `flora` no mesmo workspace.
3. Em Project Properties > Project References, marque `inet4.4`.
4. Compile o `flora` (Build Project).

Ao rodar uma configuração, a IDE abre a janela "Errors in Workspace" com três erros do validador de NED da própria IDE: dois no `QosClassifier.ned` do INET e um no `LoRaGW.ned` do FLoRa, todos em arquivos oficiais. Basta clicar em Proceed. A simulação roda normalmente, e esses erros não aparecem no Cmdenv nem no Qtenv.

### Pela linha de comando

Com o ambiente do OMNeT++ carregado (`source setenv` na pasta do OMNeT++):

```
cd inet4.4 && make makefiles && make
cd ../flora && make makefiles && make
cd simulations
../src/run_flora -f omnetpp.ini -c Comparacao_N100_LMAC_60s -r 0 -u Cmdenv
```

- `-c` escolhe a configuração e `-r` a repetição (0 a 9; use `--repeat=30` para ir além).
- `../src/run_flora -f omnetpp.ini -q configs` lista as configurações.
- Os resultados vão para `simulations/results/`.
- A repetição 0 de `Comparacao_N100_LMAC_60s` (12 h simuladas) leva cerca de 4 min num Intel Core i5-9400F.

## Script de repetições

O `simulations/rodar_comparacao_final.py` roda cada configuração até o intervalo de confiança de 95% da PLR ficar abaixo de ±1 ponto percentual, com 3 a 30 repetições. Se for interrompido, retoma de onde parou. Precisa de Linux, Python 3 e `/usr/bin/time`.

```
cd flora/simulations
python3 rodar_comparacao_final.py
python3 rodar_comparacao_final.py --configs Comparacao_N100_ALOHA_60s,Comparacao_N100_LMAC_60s
```

Sem `--configs`, ele roda as 12 configurações de comparação. As opções e as saídas estão descritas no início do script.

O script também limita a memória usada e o número de simulações ao mesmo tempo. Uma simulação de 12 h com 100 nós chegou a 8,9 GB com o LMAC-1 e 1,5 GB com o ALOHA, e o script reserva 10 GB e 2 GB para cada uma. Na monografia foi usado um i5-9400F (6 núcleos) com 16 GB de RAM, com `--ram-budget-gb 11` e `--cpu-slots 5`. Em outra máquina, ajuste esses dois valores. Com `--ram-budget-gb` abaixo de 10, as simulações do LMAC-1 não começam.

## Alterações em relação ao FLoRa 1.1.0

1. Compilação no OMNeT++ 6.3 com o INET 4.4.2 (`LoRaModulation.cc`, `run_flora`, `LoRaNic.ned`).
2. Modelo: preâmbulo de 10 símbolos, carga útil de 16 bytes no tempo no ar, sinais abaixo da sensibilidade ignorados pelo receptor, intervalo mínimo entre envios calculado pelo tempo no ar e pelo duty cycle (`LoRaTransmitter.cc`, `LoRaReceiver.cc`, `SimpleLoRaApp.cc/.ned`).
3. Módulo `Lmac1Mac`, derivado do `LoRaMac`, com o LMAC-1: DIFS de 12 CADs, backoff de 4 a 64 CADs congelado quando o canal fica ocupado, e rádio em recepção fora das transmissões.
4. Configurações das simulações (`omnetpp.ini`), script de repetições e resultados.

Cada arquivo modificado tem no cabeçalho uma linha "Modificado por André do Valle Medeiros em 2026" com o resumo da mudança.

## Resultados da monografia

| Resultado | Configurações | Repetições | Arquivo |
|---|---|---|---|
| PLR, raio de 1 km | `Comparacao_N100_{ALOHA,LMAC}_{60s,30s,15s}` | ALOHA 3/5/5, LMAC-1 3/3/3 | `results/comparacao_final_v5/resumo_comparacao_final.csv` |
| PLR, raio de 2 km | `Comparacao_AreaAmpliada_{ALOHA,LMAC}_{60s,30s,15s}` | ALOHA 6/5/5, LMAC-1 6/3/3 | `results/comparacao_area_ampliada_v5/resumo_comparacao_final.csv` |
| Posições dos nós | repetição 0 das configurações de 1 e 2 km | 1 | `results/posicoes_nos_v5.csv` |
| Teste de cobertura | `Teste_Cobertura_1_No` (7 distâncias) | 10 por distância | `results/teste_cobertura_v5/resumo_teste_cobertura.csv` |
| Terminal oculto | `TerminalOculto_{ALOHA,LMAC}_15s` | 1 | `.sca` no zip da release |

As simulações de 1 e 2 km e as do terminal oculto foram geradas com uma versão anterior do código, antes de o módulo ser renomeado para `Lmac1Mac`. Com este código, a repetição 0 de `Comparacao_N100_LMAC_60s` dá exatamente os mesmos escalares. O teste de cobertura foi gerado com este código. Os `.sca` de todas as simulações estão no arquivo `resultados_monografia_v5.zip`, anexado à release `tcc-2026`.

## Licença

Este repositório mantém a licença do FLoRa: GNU LGPL versão 3 (`LICENSE.md`), que se apoia na GPL 3 (`COPYING`). O INET não é redistribuído aqui.
