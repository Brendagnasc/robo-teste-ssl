#!/usr/bin/env bash
# bluetooth_conectar.sh
#
# Parear o HC-05 e criar a porta /dev/rfcomm0, que a ponte_serial usa como se fosse
# uma serial comum.
#
#   ./scripts/bluetooth_conectar.sh                 lista os dispositivos encontrados
#   ./scripts/bluetooth_conectar.sh 98:D3:31:XX:XX:XX   pareia e cria /dev/rfcomm0
#   ./scripts/bluetooth_conectar.sh --soltar        libera a porta
#
# O HC-05 sai de fábrica em 9600 bps, que é pouco para o nosso tráfego. Configure o
# módulo uma vez, no modo AT (segure o botão do módulo ao energizar), enviando:
#   AT+UART=38400,0,0
#   AT+NAME=RoboTeste
#   AT+PSWD="1234"
# O firmware espera exatamente 38400 no canal Bluetooth.

set -euo pipefail

CANAL_RFCOMM=0
CANAL_SPP=1     # o HC-05 publica o perfil serial no canal 1

if ! command -v bluetoothctl >/dev/null; then
  echo "Falta o bluez. Instale com: sudo apt install -y bluez"
  exit 1
fi

if [ "${1:-}" = "--soltar" ]; then
  sudo rfcomm release "$CANAL_RFCOMM" && echo "porta /dev/rfcomm$CANAL_RFCOMM liberada"
  exit 0
fi

if [ $# -eq 0 ]; then
  echo "Procurando dispositivos por 10 segundos..."
  bluetoothctl --timeout 10 scan on >/dev/null 2>&1 || true
  bluetoothctl devices
  echo
  echo "Pegue o endereço do seu módulo (costuma aparecer como HC-05) e rode:"
  echo "  $0 <endereco>"
  exit 0
fi

MAC="$1"

echo "pareando $MAC..."
bluetoothctl pair "$MAC"    || echo "aviso: pareamento ja existente ou recusado"
bluetoothctl trust "$MAC"

echo "criando /dev/rfcomm$CANAL_RFCOMM..."
sudo rfcomm release "$CANAL_RFCOMM" 2>/dev/null || true
sudo rfcomm bind "$CANAL_RFCOMM" "$MAC" "$CANAL_SPP"

ls -l "/dev/rfcomm$CANAL_RFCOMM"
echo
echo "Pronto. Teste com o robo SUSPENSO:"
echo "  python3 testes/robo_bancada.py /dev/rfcomm$CANAL_RFCOMM vel 300 300 0 --duracao 2"
echo "E o laco fechado com:"
echo "  ./build/ponte_serial --serial /dev/rfcomm$CANAL_RFCOMM --bluetooth"
