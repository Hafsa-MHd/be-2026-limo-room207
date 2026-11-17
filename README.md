# LIMO — simulation Room 207

Simulation de la détection de la chaussée, du calcul de trajectoire et du pilotage d'un robot LIMO Ackermann. La démonstration démarre du parking et envoie une destination en coordonnées métriques dans le repère `odom`.

> **Pour l’équipe (installation from zero, nettoyage de l’ancien env, démo)**  
> → lire **[GUIDE_EQUIPE.md](GUIDE_EQUIPE.md)** en premier.

## 1. Configuration nécessaire

Environnement vérifié en local : Ubuntu 22.04.5 LTS sous WSL 2, ROS 2 Humble, Gazebo Classic 11.10.2, Python 3.10.12, OpenCV 4.5.4 et NumPy 1.21.5. Une session graphique Linux ou WSLg permet d'observer Gazebo et RViz.

Les sources de l'école fournissent `room207_limo` (lancement de la simulation), `limo_car` (modèle et plugin Ackermann) et `qcar_large_track` (monde et texture de piste). Suivre [la préparation de l'environnement école](SETUP_ECOLE_GITLAB.md#2-installation-from-zero-ubuntu-2204) ou le [guide équipe](GUIDE_EQUIPE.md) pour compiler ces dépendances dans `~/ros2_ws`. Le manifeste [package.xml](limo_simple/package.xml) déclare les dépendances du package de l'équipe.

Depuis la racine du dépôt de l'équipe, une fois les dépendances école compilées :

```bash
project_dir="$(pwd)"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/ros2_ws_simple
cd ~/ros2_ws_simple
rosdep install --from-paths "$project_dir/limo_simple" --ignore-src -r -y
colcon build --base-paths "$project_dir/limo_simple" --symlink-install
source install/setup.bash
ros2 pkg prefix limo_simple
```

## 2. Architecture

- [`road_vision.py`](limo_simple/limo_simple/road_vision.py) détecte l'asphalte sur l'image de la caméra avec OpenCV. Le nœud publie un masque, des échantillons de centre de voie et un panneau de diagnostic.
- [`track_planner.py`](limo_simple/limo_simple/track_planner.py) explore des poses sur la carte de la piste. Pour chaque mouvement candidat en marche avant, il vérifie le rayon de braquage et les points de contact des pneus, puis renvoie des points à suivre. La carte est la texture du monde Gazebo, lue depuis le package `qcar_large_track`.
- [`polar_follow.py`](limo_simple/limo_simple/polar_follow.py) reçoit la destination et l'odométrie. En mode sud, il réutilise la trajectoire du couloir sud si elle dessert la destination et appelle le planificateur sur carte dans les autres cas. Il suit le trajet avec une cible glissante, commande le braquage et la vitesse, et s'immobilise à l'arrivée ou lorsqu'aucune route n'est disponible. La perception reste observable pendant le suivi sur carte.
- [`spawn_parking.py`](limo_simple/limo_simple/spawn_parking.py) initialise le départ à partir de `/robot_description` et vérifie la pose publiée sur `/odom`. Il utilise les services Gazebo `/delete_entity` et `/spawn_entity`. Le contrôleur utilise aussi ces services pour le marqueur de destination.
- [`lane_follow.py`](limo_simple/limo_simple/lane_follow.py) fournit le mode de suivi de voie par caméra, sélectionnable avec `control:=lane`. Le mode de mission utilise `control:=polar`.

**Interfaces ROS 2**

- Gazebo publie `/rgb/image_raw`, `/odom` et les transformations des roues sur `/tf`.
- La perception publie `/vision/road_ok` (`Bool`), `/vision/center` (centre, limites, orientation et biais), `/vision/black_bias` et les images `/vision/mask`, `/vision/overlay`, `/vision/panel`.
- La mission reçoit `/control/mission_goal` (`PoseStamped`) et `/control/mission_enable` (`Bool`). Le contrôleur publie `/cmd_vel` (`Twist`, vitesse en m/s sur `linear.x`, angle de braquage en rad sur `angular.z`), `/control/goal_reached`, `/control/gate_reason` et `/control/goal_marker`. En conduite, `polar_follow` est le seul émetteur de `/cmd_vel`.

**Lancements et paramètres**

- [`room207_drive.launch.py`](limo_simple/launch/room207_drive.launch.py) lance le monde, le robot, la perception et le contrôleur. L'argument `d8_south_route:=true` active le mode sud avec recherche de trajectoire sur carte. Le lancement par défaut conserve l'itinéraire nord de D8.
- [`room207_vision.launch.py`](limo_simple/launch/room207_vision.launch.py) lance la simulation et la perception pour inspecter la caméra et son masque.
- [`road_vision.yaml`](limo_simple/config/road_vision.yaml) règle la zone d'intérêt (`roi_top_frac`), le seuil d'asphalte (`black_threshold`) et la détection du marquage jaune.
- [`polar_follow.yaml`](limo_simple/config/polar_follow.yaml) règle l'empattement (0,24 m), la limite de braquage (0,45 rad), la fréquence de contrôle (20 Hz), la taille de la carte (4,88 × 6,20 m) et la tolérance d'arrivée (0,12 m). `map_texture` permet une surcharge explicite de la texture.

## 3. Démonstration

Ouvrir un seul Gazebo pour l'essai. Dans un premier terminal :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
ros2 launch limo_simple room207_drive.launch.py d8_south_route:=true
```

Une fois le robot immobile au départ, ouvrir un second terminal dans la racine du dépôt, charger les mêmes trois fichiers `setup.bash`, puis lancer :

```bash
python3 -u tools_test_control_once.py --goal 2.101 -0.644
```

La cible D5 apparaît dans Gazebo. Le robot quitte le parking, traverse la jonction sud et s'arrête à moins de 12 cm de la cible. Pour D8, utiliser `--goal 0.976 -0.897`. Pour une arrivée sur l'anneau noir du rond-point NE (démo corridor serré), utiliser `--goal 2.100 2.110`. Pour un autre point de la piste, donner ses coordonnées dans `--goal X Y`. Entre deux essais, remettre le robot au parking :

```bash
ros2 run limo_simple spawn_parking --ros-args -p tol_m:=0.04
```

Dans RViz ou `rqt_image_view`, sélectionner `/vision/panel` pour observer l'image, le masque et la surimpression. Le test affiche la sortie du parking, l'erreur finale, la proportion de trames reçues jugées exploitables et l'empiètement maximal des pneus déduit des transformations Gazebo. `RESULT PASS` exige une arrivée à moins de 12 cm, aucun recul et un empiètement maximal de 10 mm. Une destination sur asphalte peut être inaccessible en marche avant avec les contraintes du modèle, auquel cas le contrôleur publie `no_route_from_parking`.

