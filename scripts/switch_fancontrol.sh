#!/bin/bash
#
# Dieses Script kann mit "start" oder "stop" aufgerufen werden.
#
# Bei "start [<config-file>]" wird überprüft, ob die (alternative)
# Konfigurationsdatei (Standard: /etc/fancontrol) und alle darin referenzierten
# Dateien vorhanden sind. Falls ja:
#  - Falls der systemd-Service "fancontrol" aktiv ist, wird er gestoppt.
#  - Die alternative fancontrol-Instanz wird im Hintergrund gestartet.
#  - Ein Monitor-Prozess wird angelegt, der beim Beenden der manuellen Instanz,
#    sofern der Service vorher aktiv war, den originalen Service wieder startet.
#
# Bei "stop" wird die alternative fancontrol-Instanz (sofern laufend) beendet.
#
# Verwendung:
#   sudo ./switch_fancontrol.sh start [<config-file>]
#   sudo ./switch_fancontrol.sh stop

ALT_PIDFILE="/var/run/fancontrol_alt.pid"
DEFAULT_CONFIG="/etc/fancontrol"

# Hilfsfunktion: Ermittelt anhand eines relativen Pfadnamens den zu erwartenden absoluten Pfad.
resolve_path() {
    local file="$1"
    if [[ "$file" =~ ^/ ]]; then
        echo "$file"
    elif [[ "$file" =~ ^hwmon[0-9] ]]; then
        echo "/sys/class/hwmon/$file"
    elif [[ "$file" =~ ^[0-9]+-[0-9a-f]{4} ]]; then
        echo "/sys/bus/i2c/devices/$file"
    else
        echo "$file"
    fi
}

# Funktion: Prüft alle in der Konfiguration referenzierten Dateien.
check_config_files() {
    local config="$1"
    declare -A files_to_check

    # FCTEMPS: Format: pwm=TEMP
    local fctemps_line
    fctemps_line=$(grep '^FCTEMPS=' "$config")
    if [ -n "$fctemps_line" ]; then
        local fctemps_value="${fctemps_line#FCTEMPS=}"
        for token in $fctemps_value; do
            local pwm temp
            pwm=$(echo "$token" | cut -d'=' -f1)
            temp=$(echo "$token" | cut -d'=' -f2)
            files_to_check["$pwm"]=1
            files_to_check["$temp"]=1
        done
    fi

    # FCFANS: Format: pwm=FAN (ggf. mehrere FANs, hier als einzelner Eintrag angenommen)
    local fcfans_line
    fcfans_line=$(grep '^FCFANS=' "$config")
    if [ -n "$fcfans_line" ]; then
        local fcfans_value
        fcfans_value=$(echo "${fcfans_line#FCFANS=}" | sed 's/^ *//')
        for token in $fcfans_value; do
            local pwm fan
            pwm=$(echo "$token" | cut -d'=' -f1)
            fan=$(echo "$token" | cut -d'=' -f2)
            files_to_check["$pwm"]=1
            files_to_check["$fan"]=1
        done
    fi

    # Numerische Parameter: MINTEMP, MAXTEMP, MINSTART, MINSTOP, MINPWM, MAXPWM, AVERAGE
    for key in MINTEMP MAXTEMP MINSTART MINSTOP MINPWM MAXPWM AVERAGE; do
        local line
        line=$(grep "^$key=" "$config")
        if [ -n "$line" ]; then
            local value="${line#${key}=}"
            for token in $value; do
                local pwm
                pwm=$(echo "$token" | cut -d'=' -f1)
                files_to_check["$pwm"]=1
            done
        fi
    done

    local all_exist=true
    for file in "${!files_to_check[@]}"; do
        local abs_path
        abs_path=$(resolve_path "$file")
        if [ ! -e "$abs_path" ]; then
            echo "Error: Referenzierte Datei '$abs_path' existiert nicht."
            all_exist=false
        fi
    done

    if [ "$all_exist" != true ]; then
        return 1
    else
        return 0
    fi
}

# Funktion: Startet die alternative fancontrol-Instanz.
start_alt_fancontrol() {
    local config="${1:-$DEFAULT_CONFIG}"

    if [ ! -f "$config" ]; then
        echo "Error: Konfigurationsdatei '$config' existiert nicht."
        exit 1
    fi

    # Prüfe alle referenzierten Dateien.
    if ! check_config_files "$config"; then
        echo "Nicht alle referenzierten Dateien wurden gefunden. Abbruch."
        exit 1
    fi

    # Falls bereits eine alternative Instanz läuft, abbrechen.
    if [ -f "$ALT_PIDFILE" ]; then
        local pid
        pid=$(cat "$ALT_PIDFILE")
        if kill -0 "$pid" 2>/dev/null; then
            echo "Alternative fancontrol-Instanz läuft bereits (PID: $pid)."
            exit 0
        else
            sudo rm -f "$ALT_PIDFILE"
        fi
    fi

    # Prüfen, ob der systemd-fancontrol Service aktiv ist.
    local restart_original=false
    if systemctl is-active --quiet fancontrol; then
        restart_original=true
        echo "fancontrol systemd-Service ist aktiv. Stoppe den Service..."
        sudo systemctl stop fancontrol || { echo "Fehler beim Stoppen des fancontrol-Services."; exit 1; }
    else
        echo "fancontrol systemd-Service ist nicht aktiv. Es wird kein Service-Stopp bzw. -Restart durchgeführt."
    fi

    echo "Starte alternative fancontrol-Instanz mit Konfigurationsdatei: $config"
    sudo /usr/sbin/fancontrol "$config" > /tmp/fctest.log 2>&1 & # >/dev/null &
    local alt_pid=$!
    # echo "$alt_pid" > "$ALT_PIDFILE"
    echo "$alt_pid" | sudo tee "$ALT_PIDFILE" > /dev/null
    echo "Alternative fancontrol-Instanz gestartet (PID: $alt_pid)."

    # Monitor-Prozess: Wird im Hintergrund gestartet.
    # Falls der Service vorher aktiv war, wird er nach Beendigung neu gestartet.
    local restart_flag="$restart_original"
    (
        # Polling-Schleife mit einer Wartezeit von einer Sekunde
        while ps -p "$alt_pid" > /dev/null 2>&1; do
            sleep 1
        done

        echo "Alternative fancontrol-Instanz wurde beendet."
        sudo rm -f "$ALT_PIDFILE"
        if [ "$restart_flag" = true ]; then
            echo "Starte originalen fancontrol-Service neu..."
            sudo systemctl start fancontrol
        fi
        exit 0
    ) &
    echo "Skript beendet. Die alternative Instanz läuft im Hintergrund."
}

# Funktion: Stoppt die alternative fancontrol-Instanz, sofern sie läuft.
stop_alt_fancontrol() {
    if [ ! -f "$ALT_PIDFILE" ]; then
        echo "Keine alternative fancontrol-Instanz gefunden."
        exit 0
    fi

    local pid
    pid=$(cat "$ALT_PIDFILE")
    if kill -0 "$pid" 2>/dev/null; then
        echo "Beende alternative fancontrol-Instanz (PID: $pid)..."
        sudo kill "$pid"
        sleep 2
        if kill -0 "$pid" 2>/dev/null; then
            echo "Alternative fancontrol-Instanz konnte nicht beendet werden."
            exit 1
        else
            echo "Alternative fancontrol-Instanz beendet."
        fi
    else
        echo "Keine laufende alternative fancontrol-Instanz gefunden."
    fi
    sudo rm -f "$ALT_PIDFILE"
    exit 0
}

# Hauptprogramm: Auswertung der Kommandozeilenparameter
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 {start [<config-file>]|stop}"
    exit 1
fi

action="$1"
shift

case "$action" in
    start)
        start_alt_fancontrol "$@"
        ;;
    stop)
        stop_alt_fancontrol
        ;;
    *)
        echo "Unbekannte Aktion: $action"
        echo "Usage: $0 {start [<config-file>]|stop}"
        exit 1
        ;;
esac
