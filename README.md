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

cd
git clone https://github.com/ultralytics/ultralytics.git
cd ultralytics
pip install -e .
```

## Parametres

Il est possible de modifier plusieurs paramètres dans le code afin de changer le comportement du robot.
- Les paramètres de vitesse de déplacement sont définis dans le script 
basic_move
- Les paramètres de détection de l'environnement (capteur LiDAR) sont définis dans le script scan_echo
- Les transformations et le placement des markers sont effectués dans le script place_markers. Le script test_markers_to_place envoie des markers aléatoires dans le topic /markers_to_place à des fins de tests.


## Get Started

### Install

   ```bash
   git clone https://github.com/JulesLaBulle/uvlarm-Dominique.git
   cd uvlarm-Dominique
   ```

   ```bash
   colcon build
   source install/setup.bash
   ```

## Lancement des Simulations et Démos

Pour chaque Challenge, une fois que le système est lancé et initialisé, une pression sur le bouton B0 lance le déplacement du robot. Ce déplacement peut être mis en pause en soulevant le robot ou en pressant le bouton B1.

### Challenge 1

```bash
ros2 launch uvlarm-Dominique simulation_launch.yaml
ros2 launch uvlarm-Dominique launch_challenge1.yaml
```

### Challenge 2

```bash
ros2 launch uvlarm-Dominique simulation_v2_launch.yaml
ros2 launch uvlarm-Dominique launch_challenge2.yaml
```

## Structure du Projet

- **`config`** : Fichiers de configuration pour les lancements de rviz2
  - `challenge1_rviz_config.rviz` 
  - `slam_config.rviz`
  - `default.rviz` 

- **`include`** : Contient les données d'entrainement du modèle YOLO

- **`launch`** : 
  - `launch_basic_move.yaml` : Lancement du déplacement du robot
  - `launch_camera.yaml` : Lancement de la détection de pixels verts
  - `launch_challenge1.yaml` : Lancement de la démonstration pour le challenge 1
  - `simulation_launch.yaml` : Lancement de la simulation pour le Challenge 1
  
  - `simulation_v2_launch.yaml` : Lancement de la simulation pour le Challenge 2
  - `launch_challenge2.yaml` : Lancement de la démonstration pour le Challenge 2
  - `launch_vision.yaml` : Lancement de la detection des fantomes
 
- **`scripts`** : 
  - `basic_move` : Mouvement du robot
  - `camera` : Récupération et analyse des images
  - `detect_green_object` : Détection d'objets verts sans distinction (Challenge 1)
  - `detect_bottle` : Détection des bouteilles 
  - `detect_ghost` : Détection des pixels verts
  - `scan_echo` : Récupération et traitement des données du capteur LIDAR
  - `test_markers_to_place` : Envoie des marqueurs aléatoirement dans un topic pour des tests
  - `place_markers`: Réalise les transformations sur les coordonées envoyées par la camera et place les marqueurs sur rviz 


- **`CMakeLists.txt`** 
- **`package.xml`** 
- **`README.md`** dans le dossier racine du package