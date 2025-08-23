#!/usr/bin/env bash

# Pfade
MAP_FILE="/home/pi/Datentransfer/map_saves/komplettes_Labor_mit_Flur.yaml"
PARAMS_FILE="/home/pi/Datentransfer/nav2_params.yaml"

# Nodes in der Reihenfolge, wie sie hochgefahren werden sollten:
NODES=(
  "map_server nav2_map_server map_server --ros-args -p yaml_filename:=\"${MAP_FILE}\""
  "amcl      nav2_amcl        amcl"
  "planner_server  nav2_planner    planner_server --ros-args params_file:=\"${PARAMS_FILE}\""
  "controller_server nav2_controller controller_server --ros-args params_file:=\"${PARAMS_FILE}\""
  "bt_navigator      nav2_bt_navigator  bt_navigator --ros-args params_file:=\"${PARAMS_FILE}\""
)

echo "== Starte Nav2-Stack mit eigenem params_file =="
echo

# 1) jeden Node prüfen und ggf. starten
for entry in "${NODES[@]}"; do
  read -r name pkg exe args <<< "$entry"
  if ros2 node list | grep -qw "/${name}"; then
    echo "[✓] '${name}' läuft bereits."
  else
    echo "[→] Starte '${name}'..."
    ros2 run $pkg $exe $args &
    sleep 2
  fi
done

# 2) Manuelles Lifecycle-Handling
echo
echo "== Lifecycle-Management =="
for name in map_server amcl planner_server controller_server bt_navigator; do
  echo
  echo ">>> Bearbeite '/${name}'"
  state=$(ros2 lifecycle get "/${name}" 2>/dev/null | awk '{print $1}')
  case "$state" in
    unconfigured)
      echo "    unconfigured → configure & activate"
      ros2 lifecycle set "/${name}" configure
      ros2 lifecycle set "/${name}" activate
      ;;
    inactive)
      echo "    inactive → activate"
      ros2 lifecycle set "/${name}" activate
      ;;
    active)
      echo "    already active"
      ;;
    *)
      echo "    state='$state' (überspringe)"
      ;;
  esac

  # Für bt_navigator warten wir zusätzlich auf den follow_path-Action-Server
  if [ "$name" = "bt_navigator" ]; then
    echo -n "    Warte auf follow_path Action-Server..."
    # bis zu 10s warten
    timeout=0
    until ros2 action list | grep -q "/follow_path"; do
      sleep 1
      (( timeout++ ))
      if [ "$timeout" -ge 10 ]; then
        echo " fehlgeschlagen!"
        echo "    → Stelle sicher, dass 'controller_server' aktiv ist und die critics geladen sind."
        break
      fi
      echo -n "."
    done
    echo " OK"
  fi
done

echo
echo "== Fertig. Bitte prüfe mit:"
echo "   ros2 lifecycle get <node>    # z.B. /controller_server"
echo "   ros2 action list             # sollte /follow_path enthalten"
echo "   ros2 topic echo /map         # Karte sichtbar?"
echo


# Starte turtlebot3_bringup
ros2 launch turtlebot3_bringup robot.launch.py > /tmp/tb3_bringup.log 2>&1 &
echo "TurtleBot3 bringup gestartet..."
sleep 7

# Starte Nav2 mit Karte
ros2 launch nav2_bringup navigation_launch.py \
  map:=/home/pi/Datentransfer/map_saves/komplettes_Labor_mit_Flur.yaml \
  autostart:=true > /tmp/nav2.log 2>&1 &
echo "Nav2 gestartet..."
sleep 7  # Gib Nav2 Zeit zum Initialisieren

# Starte benutzerdefiniertes Skript
./nav2_startup.sh > /tmp/nav2_startup.log 2>&1 &
echo "Startup-Skript ausgeführt..."

ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \ "{map_url: '/home/pi/Datentransfer/map_saves/komplettes_Labor_mit_Flur.yaml'}"
sleep 3

# Warte, bis der map_server-Service verfügbar ist
echo "Warte auf map_server/load_map..."
until ros2 service list | grep -q "/map_server/load_map"; do sleep 1; done
sleep 7

# Lade Karte erneut (optional)
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
"{map_url: '/home/pi/Datentransfer/map_saves/komplettes_Labor_mit_Flur.yaml'}" > /tmp/loadmap.log 2>&1 &
echo "Karte geladen..."

ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
"{map_url: '/home/pi/Datentransfer/map_saves/komplettes_Labor_mit_Flur.yaml'}" > /tmp/loadmap.log 2>&1 &
echo "Karte geladen..."
sleep 5

ros2 run v4l2_camera v4l2_camera_node 2>&1 &
echo "Kamera gestartet..."

echo "Alle Prozesse wurden gestartet."
