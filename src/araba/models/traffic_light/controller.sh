#!/bin/bash

WORLD="default"
TOPIC="/traffic_light/state"

OFF_MAT="material: {emissive: {r:0, g:0, b:0, a:1}, ambient: {r:0.1, g:0.1, b:0.1, a:1}, diffuse: {r:0.1, g:0.1, b:0.1, a:1}}"
RED_ON="material: {emissive: {r:1, g:0, b:0, a:1}, ambient: {r:1, g:0, b:0, a:1}, diffuse: {r:1, g:0, b:0, a:1}}"
YELLOW_ON="material: {emissive: {r:1, g:1, b:0, a:1}, ambient: {r:1, g:1, b:0, a:1}, diffuse: {r:1, g:1, b:0, a:1}}"
GREEN_ON="material: {emissive: {r:0, g:1, b:0, a:1}, ambient: {r:0, g:1, b:0, a:1}, diffuse: {r:0, g:1, b:0, a:1}}"

update_visual() {
  local COLOR_NAME=$1
  local MAT_DATA=$2
  gz service -s /world/$WORLD/visual_config \
    --reqtype gz.msgs.Visual \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "name: 'traffic_light::link::${COLOR_NAME}', ${MAT_DATA}"
}

turn_off_all() {
  update_visual "red" "$OFF_MAT"
  update_visual "yellow" "$OFF_MAT"
  update_visual "green" "$OFF_MAT"
}

echo "========================================="
echo " 🚥 HARICI TRAFIK ISIGI KONTROLCUSU AKTIF"
echo "========================================="

while true; do
  turn_off_all
  update_visual "green" "$GREEN_ON"
  gz topic -t $TOPIC -m gz.msgs.StringMsg -p "data: 'GREEN'"
  echo "[$(date +'%H:%M:%S')] -> YESIL"
  sleep 5

  turn_off_all
  update_visual "yellow" "$YELLOW_ON"
  gz topic -t $TOPIC -m gz.msgs.StringMsg -p "data: 'YELLOW'"
  echo "[$(date +'%H:%M:%S')] -> SARI"
  sleep 2

  turn_off_all
  update_visual "red" "$RED_ON"
  gz topic -t $TOPIC -m gz.msgs.StringMsg -p "data: 'RED'"
  echo "[$(date +'%H:%M:%S')] -> KIRMIZI"
  sleep 5
done