# Guide équipe — installer l’environnement à jour et rejouer la démo LIMO

Ce dépôt contient **notre stack perception + planification + contrôle** (`limo_simple`) pour la Room 207.  
Les enseignants ont changé / rafraîchi l’environnement école : il faut **enlever l’ancienne version**, **recloner le nouvel environnement GitLab**, puis installer **ce dépôt** par-dessus.

**PC cible :** Windows + **WSL Ubuntu 22.04** + ROS 2 **Humble** + Gazebo Classic **11**  
(ne pas utiliser Jazzy / Ubuntu 24 pour Room 207)

Compte école : [GitLab Université de Lorraine](https://gitlab.univ-lorraine.fr)  
Groupe : [robotic_lab](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab)

---

## 0. Prérequis (une seule fois)

Dans WSL (`Ubuntu-22.04`) :

```bash
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null && echo "ROS Humble OK"
gazebo --version   # Classic 11.x
```

Si ROS / Gazebo ne sont pas installés, suivre le tutoriel Humble officiel, puis :

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge \
  ros-humble-joint-state-publisher python3-opencv python3-numpy \
  python3-pil.imagetk git
sudo rosdep init 2>/dev/null || true
rosdep update
```

Auth GitLab (HTTPS) : un token ou login UL quand `git clone` le demande.

---

## 1. Tout arrêter (anciennes sims)

```bash
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f rviz2 || true
pkill -9 -f room207 || true
pkill -9 -f polar_follow || true
pkill -9 -f road_vision || true
pkill -9 -f limo_simple || true
```

---

## 2. Supprimer l’ancienne installation

> Ça efface les clones école et les builds locaux. Garde une copie ailleurs si tu as des notes perso.

```bash
# Ancien workspace école
rm -rf ~/ros2_ws

# Ancien workspace équipe (perception)
rm -rf ~/ros2_ws_simple

# Anciennes copies éparpillées (adapte les chemins si besoin)
rm -rf ~/limo ~/room207-limo ~/environment
rm -rf ~/Downloads/lmo ~/Downloads/limo_simple 2>/dev/null || true
```

**Ne pas** réutiliser un vieux dump GitHub type `lmo` / home WSL archivé : ce n’est pas l’environnement école à jour.

---

## 3. Recloner l’environnement école (GitLab) et compiler

```bash
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/environment.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/teleoperation.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/recording.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/students/be-2026-groupe-4/room207-limo.git

# Lien attendu par les packages école
ln -sfn limo/ros2_humble/limo_ros2 limo_ros2

# Ignorer les arbres Foxy inutiles sous Humble
touch limo/ros2_foxy/COLCON_IGNORE 2>/dev/null || true

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Vérifier la simu école seule (sans notre perception)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch room207_limo room207_limo.launch.py
```

Tu dois voir Gazebo Room 207, le LIMO, `/rgb/image_raw`, `/odom`.  
Si ça échoue, **ne passe pas à l’étape 4** : règle d’abord l’env école (réseau GitLab, build, affichage WSLg).

Plus de détail : [SETUP_ECOLE_GITLAB.md](SETUP_ECOLE_GITLAB.md).

---

## 4. Cloner ce dépôt (stack équipe) et compiler

```bash
# Choisis un dossier clair (exemple)
mkdir -p ~/be_limo
cd ~/be_limo
git clone https://github.com/Hafsa-MHd/be-2026-limo-room207.git
cd be-2026-limo-room207

# Corriger les fins de ligne Windows → Linux si besoin
sed -i 's/\r$//' tools_*.sh 2>/dev/null || true
chmod +x tools_*.sh 2>/dev/null || true

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

mkdir -p ~/ros2_ws_simple/src
ln -sfn "$(pwd)/limo_simple" ~/ros2_ws_simple/src/limo_simple

cd ~/ros2_ws_simple
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws_simple/install/setup.bash

ros2 pkg prefix limo_simple
```

À chaque ouverture de terminal :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash
```

---

## 5. Voir la même démo (ce que tu vois en séance)

### 5.1 Lancer la stack complète

Depuis **la racine de ce dépôt** :

```bash
cd ~/be_limo/be-2026-limo-room207
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws_simple/install/setup.bash

ros2 launch limo_simple room207_drive.launch.py d8_south_route:=true
```

Attends que Gazebo soit stable et que le robot soit au parking (ou force le parking) :

```bash
# autre terminal, mêmes 3 source
bash tools_force_parking_now.sh
```

### 5.2 Un seul objectif (ex. rond-point NE)

```bash
python3 -u tools_test_control_once.py --goal 2.100 2.110
```

Autres points utiles :

| Nom | Commande |
|-----|----------|
| D5 | `--goal 2.101 -0.644` |
| D8 | `--goal 0.976 -0.897` |
| Rond-point (anneau noir) | `--goal 2.100 2.110` |

`RESULT PASS` = arrivée ≤ 12 cm, pas de marche arrière, empiètement pneus ≤ 10 mm.

### 5.3 Boucle multi-destinations (démo profs)

Si Gazebo tourne déjà avec la stack :

```bash
bash tools_run_multi_goals.sh
```

Si tout est cassé / plus de `/odom` :

```bash
bash tools_rerun_live_demo.sh
```

Le point **02** est le rond-point `(2.100, 2.110)`. Le robot repart du parking entre chaque but.

### 5.4 Observer la perception

Dans RViz ou `rqt_image_view` : topic `/vision/panel`.

---

## 6. Mettre à jour plus tard

### Environnement école

```bash
cd ~/ros2_ws/src/environment && git pull
cd ~/ros2_ws/src/limo && git pull
cd ~/ros2_ws/src/room207-limo && git pull
# (+ teleoperation / recording si besoin)
cd ~/ros2_ws && colcon build --symlink-install
```

### Notre stack

```bash
cd ~/be_limo/be-2026-limo-room207
git pull
cd ~/ros2_ws_simple && colcon build --symlink-install
```

---

## 7. Dépannage rapide

| Symptôme | Action |
|----------|--------|
| Pas de `/odom` après un reset | `bash tools_force_parking_now.sh` (téléport). Si échec : `bash tools_rerun_live_demo.sh` |
| `no_route_from_parking` | Destination hors asphalte accessible en marche avant ; changer le `--goal` |
| Gazebo sans fenêtre | WSLg / DISPLAY ; relancer `gzclient` ou le launch |
| Mauvais ROS | Vérifier `echo $ROS_DISTRO` → `humble` |
| Build `limo_simple` KO | `source ~/ros2_ws/install/setup.bash` **avant** de builder `ros2_ws_simple` |
| Ancien code qui revient | Vérifier que `~/ros2_ws_simple/src/limo_simple` pointe bien vers **ce** clone (`readlink -f …`) |

---

## 8. Architecture (rappel)

- `road_vision` — asphalte caméra → `/vision/*`
- `track_planner` — routes marche avant sur la texture piste
- `polar_follow` — mission `/control/mission_goal` → `/cmd_vel`
- `spawn_parking` / `tools_force_parking_now.sh` — départ parking officiel

Détails techniques : [README.md](README.md).

---

## 9. Règles d’équipe

- Ne **pas** pousser sur GitLab école sans accord explicite.
- Ne **pas** mélanger un vieux workspace avec le nouveau : un seul `~/ros2_ws` + un seul `~/ros2_ws_simple`.
- En cas de doute : refaire les sections **1 → 4**.
