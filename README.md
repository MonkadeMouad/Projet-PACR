# README - Projet PACR : Planification et Suivi de Chemin

## Introduction
Le projet **PACR** (Planification, Autonomie, Contrôle et Robotique) vise à développer une architecture de navigation autonome pour un robot mobile simulé sous **Gazebo**. L'objectif est de permettre au robot de collecter, transformer et livrer des objets tout en évitant les obstacles et un robot concurrent.

## TP1 : Planification de Chemin
Ce TP consistait à implémenter un **nœud ROS2** répondant au service `get_path`, qui génère un chemin entre deux positions en évitant les obstacles.

### 1. **Conception**
- Utilisation de la carte d'occupation du topic `/costmap`.
- Séparation entre l'algorithme de planification et le nœud ROS2.
- Implémentation d'une classe de planification exploitant `GridMap` de `map_utils`.
- Utilisation de l'algorithme **RRT*** pour assurer une planification anytime.

### 2. **Implémentation**
- Création du package **`pacr_solutions`** dépendant de `pacr_interfaces`, `pacr_simulation` et `nav_msgs`.
- Création du nœud ROS2 `path_planning.py` qui fournit un service `get_path` pour calculer un chemin.
  
### 3. **Tests**
- Utilisation du fichier launch `test_path_planning.launch.xml` pour tester `get_path`.
- Création d'un fichier launch propre à `pacr_solutions`.
- Tests avec **RViz** :
  - Définition des poses de départ et d'objectif.
  - Vérification des trajectoires et ajustements si nécessaire.

## TP2 : Suivi de Chemin
Ce TP consiste à implémenter un **nœud ROS2** répondant au service `get_cmd`, permettant au robot de suivre un chemin tout en évitant les obstacles dynamiques.

### 1. **Conception**
- Implémentation d'un **contrôleur réactif** pour suivre le chemin.
- Utilisation des données de `/costmap` et `/scan` pour l’évitement d’obstacles.
- Souscription aux topics de position et de vitesse du robot.

### 2. **Implémentation**
- Ajout du service `get_cmd` dans `pacr_solutions`.
- Création d’un nœud ROS2 `controller_node.py`.
- Gestion de la génération des commandes de vitesse pour le robot.
  
### 3. **Tests**
- Utilisation du fichier launch `test_goto.launch.xml`.
- Tests avec **RViz** :
  - Définition des poses de départ et d'objectif.
  - Vérification du suivi de chemin et des corrections d’évitement.

## TP3 : Planification de Tâches
Ce TP vise à implémenter un **nœud ROS2** répondant au service `get_action`, permettant au robot de décider des actions à entreprendre en fonction de l'état de l'environnement et de sa mission.

### 1. **Conception**
- Modélisation de la tâche sous la forme d'un **Processus de Décision Markovien (MDP)**, défini par le tuple <S, A, T, R> :
  - **S** : États possibles du robot et de l'environnement.
  - **A** : Actions que le robot peut entreprendre.
  - **T** : Modèle de transition entre les états.
  - **R** : Fonction de récompense associée aux transitions.
- Utilisation de l'**itération sur la valeur** pour déterminer une politique optimale π*: S → A.
- Intégration de la politique dans le service `get_action` pour fournir des actions en temps réel.

### 2. **Implémentation**
- Création d'un nœud ROS2 `task_planner_node.py` dans le package `pacr_solutions`.
- Implémentation du service `get_action` :
  - Souscription aux topics nécessaires pour obtenir l'état courant du robot et de l'environnement.
  - Calcul de l'action optimale en fonction de la politique dérivée du MDP.
  - Renvoi de l'action au nœud d'exécution pour réalisation.

### 3. **Tests**
- Utilisation d'un fichier launch spécifique pour intégrer le nœud de planification de tâches avec les autres composants du système.
- Scénarios de test :
  - Simulation de différentes configurations d'environnement et d'états du robot.
  - Validation de la cohérence des actions proposées avec les objectifs de la mission.
  - Vérification de l'adaptabilité du planificateur face aux changements dynamiques de l'environnement.

Ce TP finalise l'architecture de contrôle du robot, en intégrant la planification de tâches haut niveau avec la planification de chemin et le suivi de trajectoire, permettant ainsi une navigation autonome efficace dans l'entrepôt simulé. 