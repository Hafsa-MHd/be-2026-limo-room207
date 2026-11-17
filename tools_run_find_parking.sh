#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
python3 "$ROOT/tools_find_true_parking.py"
# also sample a few known bay poses + white-gap risk
python3 - <<'PY'
import cv2, numpy as np
TEX="/home/ha054/ros2_ws/src/environment/qcar_large_track/models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
SX,SY=4.88,6.2
img=cv2.imread(TEX,0); h,w=img.shape
bright=(img>160).astype(np.uint8)
safe=(bright==0).astype(np.uint8)*255
dist=cv2.distanceTransform(safe,cv2.DIST_L2,5)
mpp=SX/(w-1)
for name,x,y in [
 ("bay_guess",-1.15,1.45),("bay2",-1.35,1.20),("bay3",-0.95,1.60),
 ("gap_white",-0.70,0.85),("official_road",-0.725,-0.087),
 ("now_ish",-0.70,0.86),
]:
 col=int(round((x/SX+0.5)*(w-1))); row=int(round((0.5-y/SY)*(h-1)))
 g=float(np.median(img[row-2:row+3,col-2:col+3]))
 dm=float(dist[row,col])*mpp
 print("%s (%.2f,%.2f) gray=%.0f margin=%.2fm"%(name,x,y,g,dm))
PY
