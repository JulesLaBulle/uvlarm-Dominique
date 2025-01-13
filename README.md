# UV LARM - Robot n°28

Nous avons fait le choix de nommer notre robot Dominique afin de faciliter nos conversations à son sujet.

## Prérequis

### Logiciels et outils nécessaires

**Packages requis** :
   - `colcon-common-extensions`
   - `rviz2`
   - `gazebo_ros`
**Dépendances supplémentaires** :
   - Python (avec `numpy`, `opencv-python`, et `scikit-learn` installés).


### Installation des dépendances

```bash
sudo apt update && sudo apt install -y ros-humble-rviz2 ros-humble-gazebo-ros-pkgs python3-numpy python3-opencv python3-sklearn
```

## Récupération du Projet

   ```bash
   git clone <git@github.com:JulesLaBulle/uvlarm-Dominique.git>
   ```

   ```bash
   cd <uvlarm-Dominique>
   ```

## Construction et Sourcing

   ```bash
   colcon build
   ```

   ```bash
   source install/setup.bash
   ```

## Lancement des Simulations et Démos

### Simulation avec RViz

```bash
ros2 launch uvlarm-Dominique simulation_launch.yaml
```


### Démonstration avec Turtlebot

   ```bash
   ros2 launch uvlarm-Dominique launch_challenge1.yaml
   ```

## Structure du Projet
- **`config`** : 
  - `launch/simulation_launch.yaml` 
  - `launch/launch_challenge1.yaml` 
- **`launch`** : 
  - `launch/simulation_launch.yaml` 
  - `launch/launch_challenge1.yaml` 
- **`scripts`** : 
  - `launch/simulation_launch.yaml` 
  - `launch/launch_challenge1.yaml` 
- **`CMakeLists.txt`** : 
- **`package.xml`** : 
- **`README.md`** : 

