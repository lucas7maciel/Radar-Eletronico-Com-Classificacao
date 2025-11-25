# Radar eletronico (Zephyr)

## Introducao e Objetivos
Radar didatico com deteccao/classificacao, display colorido (ANSI) e camera simulada via ZBUS. Pensado para `mps2_an385`, mas roda bem em `qemu_cortex_m3`/`native_sim` para desenvolvimento. Foco: consolidar RTOS (threads, sincronizacao, comunicacao), drivers (GPIO, display) e configuracao via Kconfig.

## Descricao Funcional
- Deteccao de passagem/velocidade: sensores magneticos simulados (GPIOs 5 e 6 no QEMU) detectam a passagem e calculam a velocidade.
- Classificacao de veiculo: contagem de pulsos no GPIO 5 (eixos) — 2 pulsos: Leve; 3 ou mais: Pesado.
- Deteccao de infracao: compara a velocidade com limites configuraveis (leves/pesados).
- Exibicao no visor com cores: display dummy + ANSI — verde (normal), amarelo (alerta, ex.: >=90%), vermelho (infracao); mostra tipo e limite aplicado.
- Captura simulada: em infracao (vermelho), aciona camera via ZBUS.
- Validacao de placa: placa Mercosul gerada pela camera simulada e validada; falhas simuladas com taxa configuravel.

## Design Tecnico
- Plataforma: mps2_an385; SO: Zephyr RTOS; APIs: GPIO, Display Dummy, ZBUS, Kconfig.
- Threads: controle (orquestra, aplica limites/alerta, aciona camera, valida placa); sensores (ISR/maquina de estados para eixos e delta, envia por fila); display (formata linha ANSI); camera/LPR (espera trigger de infracao via ZBUS, processa, gera placa valida ou nao).
- Comunicacao: filas de mensagens sensores->controle->display; ZBUS para requisicoes/respostas da camera.
- Simulacao de hardware: sensores magneticos nos GPIOs 5/6; camera via software/ZBUS; display via driver dummy + ANSI no console QEMU.

## Configuracao (Kconfig)
- CONFIG_RADAR_SENSOR_DISTANCE_MM: distancia entre sensores (mm).
- CONFIG_RADAR_SPEED_LIMIT_LIGHT_KMH: limite para veiculos leves (km/h).
- CONFIG_RADAR_SPEED_LIMIT_HEAVY_KMH: limite para veiculos pesados (km/h).
- CONFIG_RADAR_WARNING_THRESHOLD_PERCENT: percentual do limite que aciona alerta (ex.: 90).
- CONFIG_RADAR_CAMERA_FAILURE_RATE_PERCENT: chance de a camera simular falha (0-100).

## Arquivos
- `CMakeLists.txt`: integra as fontes (app + logica compartilhada).
- `prj.conf`: configuracoes gerais e valores padrao das opcoes Kconfig do radar.
- `Kconfig`: opcoes do radar (distancia, limites, alerta, falha de camera).
- `src/main.c`: threads (sensores simulados, controle, display, camera via ZBUS).
- `src/radar_logic.c`, `include/radar.h`: funcoes puras reutilizaveis e testaveis.
- `tests/unit/*`: ztests (calculo de velocidade, classificacao, validacao de placa).

## Como compilar e executar (QEMU)
Assumindo workspace Zephyr em `~/zephyrproject` e o app neste diretorio (WSL):
```sh
cd ~/zephyrproject
source zephyr/zephyr-env.sh   # ou o script que voce usa para ativar
west build -b qemu_cortex_m3 -p always /mnt/c/Users/Cacacalciano/Desktop/Projetos/Projeto-Embarcados
west build -t run
```
Se sua arvore tiver a placa `mps2_an385`, troque `-b qemu_cortex_m3` por `-b mps2_an385`.

## Como rodar os testes (ztest)
Executar a suite unit em `native_sim`:
```sh
cd ~/zephyrproject
source zephyr/zephyr-env.sh   # se ainda nao estiver ativo
west twister -T /mnt/c/Users/Cacacalciano/Desktop/Projetos/Projeto-Embarcados/tests/unit -p native_sim
```
Resultados ficam em `twister-out/`. Use `--clobber` para limpar resultados anteriores, se precisar.

## Threads e fluxo
- Sensores (sensor_sim_thread): simula pulsos nos sensores A/B, contando eixos e tempo entre sensores.
- Controle: classifica (leve/pesado), calcula velocidade com a distancia configurada, aplica limites/alerta e aciona camera em caso de infracao.
- Camera (ZBUS): ouve `camera_request_chan`, simula captura/LPR com taxa de falha configuravel e publica em `camera_response_chan`.
- Display: consome fila `display_queue` e imprime linha com cor ANSI (verde/amarelo/vermelho) e placa quando disponivel.

## Estrategia de Testes
- Unidade (ztest): calculo de velocidade; classificacao leve/pesado; validacao de placa Mercosul.
- Integracao (semi): fluxo completo simulando pulsos/velocidade e checando classificacao; comunicacao ZBUS.
- Manual/visual: execucao no QEMU observando display dummy; verificar cores (verde/amarelo/vermelho) para velocidades normal/alerta/infracao.

## Entregaveis
- Repositorio Git.
- README detalhado com descricao, instrucoes de build/execucao no QEMU, opcoes Kconfig, arquitetura (classificacao e display) e instrucoes de teste.

## Alinhamento com criterios
- Qualidade de codigo: modular, limpo.
- Criatividade: maquina de estados para classificacao e feedback visual colorido.
- Testes automaticos: ztest nas logicas criticas.
- Uso de Zephyr: multithreading, Kconfig, GPIO/display, ZBUS.
- Uso de Git: branches e commits significativos (prazo ate 25/11).

## Observacoes
- A simulacao usa ANSI no console como um "display dummy". Para hardware real, mapeie GPIOs e display no device tree e conecte as ISRs aos sensores fisicos.
- Se `mps2_an385` nao existir na sua arvore do Zephyr (foi removido em revisoes recentes), use `qemu_cortex_m3` ou troque para uma revisao do Zephyr que inclua o board.
