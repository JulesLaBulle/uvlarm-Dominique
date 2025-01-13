# UV LARM - Robot n°28

Nous avons fait le choix de nommer notre robot Dominique afin de faciliter nos conversations à son sujet.

## Auteurs
- Ruying Ji
- Jules Ramaen
- Guillaume Toupance

## Prérequis

### Logiciels et outils nécessaires

**Packages requis** :
   - `colcon-common-extensions`
   - `rviz2`
   - `gazebo_ros`

**Dépendances supplémentaires** :
   - Python (avec `numpy`, `opencv-python`, et `scikit-learn` installés).


### Installation des dépendances

Suivre les instructions d'installation de ros2 : 
https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html

```bash
sudo apt update && sudo apt install -y ros-iron-rviz2 ros-iron-gazebo-ros-pkgs python3-numpy python3-opencv python3-sklearn

```

## Récupération du Projet

   ```bash
   git clone https://github.com/JulesLaBulle/uvlarm-Dominique.git
   cd uvlarm-Dominique
   ```

## Construction et Sourcing

   ```bash
   colcon build
   source install/setup.bash
   ```

## Lancement des Simulations et Démos

### Simulation avec RViz

```bash
ros2 launch uvlarm-Dominique simulation_launch.yaml
```
### Démonstration 

   ```bash
   ros2 launch uvlarm-Dominique launch_challenge1.yaml
   ```

## Structure du Projet

- **`config`** : Fichiers de configuration pour les lancements de rviz2
  - `challenge1_rviz_config.rviz` 
  - `slam_config.rviz` 

- **`launch`** : 
  - `launch_basic_move.yaml` : Lancement du déplacement du robot
  - `launch_camera.yaml` : Lancement de la détection de pixels verts
  - `launch_challenge1.yaml` : Lancement de la démonstration pour le challenge 1
  - `launch_slam.yaml` : Lancement du robot avec cartographie dans rviz2
  - `simulation_launch.yaml` : Lancement de la simulation pour le Challenge 1

- **`scripts`** : 
  - `basic_move` : Mouvement du robot
  - `camera` : Récupération et analyse des images
  - `detect_bottle` : Détection des bouteilles (IA)
  - `detect_ghost` : Détection des pixels verts
  - `scan_echo` : Récupération et traitement des données du capteur LIDAR
  - `test_markers` : Placeur de marqueurs arbitraires sur rviz pour des tests

- **`CMakeLists.txt`** 
- **`package.xml`** 
- **`README.md`** dans le dossier racine du package

