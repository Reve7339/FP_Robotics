#!/bin/bash
# Inicia el servidor de la interfaz web
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "Iniciando servidor web para la interfaz de KiraOne en http://localhost:8080..."

# Intentar abrir el navegador por defecto
if command -v xdg-open &> /dev/null; then
  (sleep 1 && xdg-open "http://localhost:8080") &
elif command -v sensible-browser &> /dev/null; then
  (sleep 1 && sensible-browser "http://localhost:8080") &
fi

python3 -m http.server -d "$SCRIPT_DIR/ui" 8080
