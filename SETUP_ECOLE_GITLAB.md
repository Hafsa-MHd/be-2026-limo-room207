# Environnement école propre (GitLab) — avant perception

Compte : [GitLab Université de Lorraine](https://gitlab.univ-lorraine.fr)  
Groupe : [robotic_lab](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab)  
Groupe étudiant : [BE 2026 Groupe 4](https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/students/be-2026-groupe-4)

**PC cible :** WSL **Ubuntu 22.04** + ROS 2 **Humble** + Gazebo Classic 11  
(ne pas utiliser la distro Jazzy pour Room 207)

---

## 1. Dépôts à cloner (école seulement)

| Rôle | URL GitLab |
|---|---|
| Monde Room 207 / piste | https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/environment.git |
| Robot LIMO (drivers, URDF, Gazebo) | https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git |
| Téléopération | https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/teleoperation.git |
| Recording | https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/recording.git |
| Démo Room 207 (votre fork groupe) | https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/students/be-2026-groupe-4/room207-limo.git |
| Infos groupe (optionnel) | https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/students/be-2026-groupe-4/group-informations.git |

Dans `environment/` tu dois voir :

- `ensem_room207` — **monde par défaut** Room 207 (salle + piste)
- `qcar_large_track` — piste seule
- `agilex_sand_table` — table AgileX

---

## 2. Installation from zero (Ubuntu-22.04)

```bash
# Terminal WSL Ubuntu-22.04
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

cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y python3-pil.imagetk

colcon build --symlink-install
source install/setup.bash
```

Si `room207-limo` a le layout récent :

```bash
# optionnel : raccourcis école
echo 'source ~/ros2_ws/src/room207-limo/ros2_foxy/room207.bash' >> ~/.bashrc
source ~/.bashrc
r207_help
```

---

## 3. Vérifier l’environnement (sans perception)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Simu Room 207 (Gazebo + RViz)
ros2 launch room207_limo room207_limo.launch.py
# ou, si les raccourcis sont chargés :
# r207_sim
```

À ce stade tu dois voir : Gazebo Room 207, LIMO spawné, caméra `/rgb/image_raw`, lidar `/scan`.  
**Pas** de package `limo_simple` / perception tant que ça n’est pas stable.

---

## 4. Déjà présent sur ton PC (WSL Ubuntu-22.04)

Les clones école sont déjà dans `~/ros2_ws/src` avec les bons remotes GitLab.  
Pour **rafraîchir** sans tout recommencer :

```bash
cd ~/ros2_ws/src/environment && git pull
cd ~/ros2_ws/src/limo && git pull
cd ~/ros2_ws/src/teleoperation && git pull
cd ~/ros2_ws/src/recording && git pull
cd ~/ros2_ws/src/room207-limo && git pull
cd ~/ros2_ws && colcon build --symlink-install
```

---

## 5. Ne pas mélanger

- Ne pas cloner https://github.com/Ibtissam567/lmo (dump home WSL, pas un env propre)
- Ne pas lancer la perception (`limo_simple`) tant que `r207_sim` / `room207_limo` n’est pas OK
- Pas de push GitLab sans confirmation explicite
