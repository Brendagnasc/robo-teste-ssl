#!/usr/bin/env bash
# rodar_gazebo.sh
#
# Sobe o Gazebo com o robô simulado e liga nele a ponte_serial de verdade, a mesma que
# vai comandar o robô real. Ctrl+C, ou fechar a janela, encerra os dois.
#
#   ./simulacao/rodar_gazebo.sh                       com janela
#   ./simulacao/rodar_gazebo.sh --sem-janela          só o servidor, mais leve
#   ./simulacao/rodar_gazebo.sh --sem-ponte           só o robô, para comandar com o robo_bancada.py
#   ./simulacao/rodar_gazebo.sh -- --latencia 0.060   o que vem depois de -- vai para a ponte
#
# Placa de vídeo no WSL: o OpenGL usa a Intel integrada por padrão. Para usar a NVIDIA:
#   MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./simulacao/rodar_gazebo.sh

set -euo pipefail

RAIZ=$(cd "$(dirname "$0")/.." && pwd)
SERIAL=/tmp/robo_teste_serial    # criado pelo plugin; tem que bater com <serial> no .sdf
MODO=()
COM_PONTE=1
while [ $# -gt 0 ]; do
    case "$1" in
        --sem-janela) MODO=(-s); shift ;;
        --sem-ponte) COM_PONTE=0; shift ;;
        --) shift; break ;;
        *) echo "opção desconhecida: $1 (as opções da ponte vão depois de --)"; exit 1 ;;
    esac
done

cmake -S "$RAIZ" -B "$RAIZ/build" > /dev/null
cmake --build "$RAIZ/build" > /dev/null
if [ ! -f "$RAIZ/build/gazebo/librobo_simulado.so" ]; then
    echo "o plugin robo_simulado não foi construído: o Gazebo Harmonic (gz-sim8) está instalado?"
    exit 1
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="$RAIZ/build/gazebo${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

GZ="" PONTE=""
encerrar() {
    # SIGINT, e não SIGTERM, no gz sim: ele só repassa o INT para o servidor e a janela.
    # Com TERM os dois ficam rodando órfãos e a próxima simulação acha dois servidores.
    [ -n "$PONTE" ] && kill "$PONTE" 2>/dev/null
    [ -n "$GZ" ] && kill -INT "$GZ" 2>/dev/null
    wait 2>/dev/null || true
}
trap encerrar EXIT
trap 'exit 130' INT TERM

rm -f "$SERIAL"
gz sim "${MODO[@]}" -r "$RAIZ/simulacao/robo_teste.sdf" &
GZ=$!
for _ in $(seq 150); do
    [ -e "$SERIAL" ] && break
    kill -0 "$GZ" 2>/dev/null || { echo "o Gazebo fechou antes de criar a serial"; exit 1; }
    sleep 0.1
done
[ -e "$SERIAL" ] || { echo "o Gazebo não criou $SERIAL em 15 s"; exit 1; }

if [ "$COM_PONTE" = 0 ]; then
    # Um programa só por vez na serial, como no robô real
    echo "robô simulado no ar, sem a ponte: comande pela serial $SERIAL (Ctrl+C encerra)"
    wait "$GZ" || true
    exit 0
fi

# Visão em milímetros e com 40 ms de atraso, como o plugin manda; roda no máximo a
# 0,9 m/s, o <v_max_m_s> do .sdf (V_MAX_MM_S); e parada a 0,11 m da bola (raio do robô
# + raio da bola). Opções repetidas depois de -- valem por cima destas.
"$RAIZ/build/ponte_serial" --serial "$SERIAL" --milimetros --latencia 0.040 \
    --v-roda-max 0.9 --dist-parada 0.11 "$@" &
PONTE=$!
wait -n "$GZ" "$PONTE" || true
