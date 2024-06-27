<p align="center">
<img src="https://upload.wikimedia.org/wikipedia/commons/c/ca/LinkedIn_logo_initials.png" height="128">
  <h2 align="center"><a href="https://www.linkedin.com/in/cheniki-faraj-%F0%9F%91%A8%E2%80%8D%F0%9F%92%BB-575a352b7/">LinkedIn</a></h2>
</p>

<br>


![](https://i.imgur.com/waxVImv.png)


## Projet N°5 🔄 🔐
# 🏛️✨ Création d'un robot mobile autonome. ✨🏛️
# Projet de Fin d'Études de licence en MECATRONIQUE 🚀🤖



![Robot de face](https://github.com/FarajDEV/LabVIEW-ConvertisseurRomainsArabes-Authentification/assets/88864407/d1aa995f-5d5a-4c05-ac2c-bd312283926b)

Ce projet de fin d'étude en licence mécatronique explore la conception et l'implémentation d'un robot mobile autonome capable de navigation et de cartographie en utilisant principalement comme Software 🌟 **Arduino** 🌟, 🌟 **Python** 🌟, 
 et 🌟 **Matlab** 🌟. 
 Ce README documente les composants, le fonctionnement, et les étapes de développement de ce projet ambitieux.

## Table des matières

- **Introduction** 🌍🔍
  - Objectif du Projet 🎯
  - Problématiques Abordées 🛠️

- **Étude Fonctionnelle du Robot** 🤖📊
  - Exploration Autonome de l'Environnement 🚀
  - Construction d'une Carte 2D 🗺️
  - Localisation en Temps Réel 📍
  - Rendu Graphique de l'Évolution 📈
  - Navigation vers un Point Donné 🗺️📍

- **Choix des Composants et Matériels Électroniques** ⚙️🔌
  - Arduino MEGA 💡
  - Moteur Pas à Pas 🔄
  - Lidar LD06 🌐
  - Moteur Driver A4988 🚗
  - ESP32 pour Connexion sans Fil 📶

- **Conception Mécanique du Robot Mobile** 🛠️🤖
  - La conception mécanique de mon robot a été réalisée avec 🌟 **Catia V5** 🌟 📐
  - La conception mécanique a été optimisée pour que tous les éléments soient bien intégrés dans le robot, sans perte d'espace. 🔄
  - Je tiens à souligner que j'ai dû retirer un étage du robot mobile après son impression, car le moteur pas à pas utilisé avait un couple très faible, ce qui ralentissait considérablement son avancement.

- **Programmation et Logiciels Utilisés** 💻📊
  - Programmation sur Arduino 🖥️
  - Utilisation de Python pour la Visualisation 🐍📊
  - Communication avec MATLAB pour le Traitement des Données Lidar 📊🌐



### Objectifs du Projet 🎯

L'objectif principal est de développer un robot autonome capable d'explorer de manière autonome un environnement inconnu, de cartographier cet environnement en 2D avec précision, de se localiser en temps réel, de naviguer efficacement vers des points spécifiques et de rendre compte graphiquement de son évolution.

### Problématiques Abordées 🛠️

Les défis techniques incluent la perception de l'environnement via le Lidar LD06, la planification de trajectoires, la localisation précise du robot et la représentation graphique des données. Ce projet détaille les solutions apportées à ces défis à travers une approche intégrée de matériel et de logiciel.

## Étude Fonctionnelle du Robot 🤖📊

Le robot est conçu pour fonctionner de manière autonome en utilisant des capteurs pour détecter les obstacles, construire une carte 2D de son environnement et naviguer efficacement.

### Exploration Autonome de l'Environnement 🚀

Le robot utilise des capteurs pour éviter les obstacles et explorer l'environnement sans intervention humaine.

### Construction d'une Carte 2D 🗺️

En explorant, le robot construit une carte précise de l'environnement, essentielle pour la navigation autonome et la mémorisation des caractéristiques environnementales.

### Localisation en Temps Réel 📍

La localisation précise en temps réel est assurée par une fusion de données du lidar ld06 et des techniques d'odométrie, permettant au robot de se positionner avec exactitude sur la carte 2D.

### Rendu Graphique de l'Évolution 📈

Les données sont visuellement représentées pour suivre en temps réel le déplacement du robot et diagnostiquer tout problème éventuel.

### Navigation vers un Point Donné 🗺️📍

Le robot est capable de naviguer efficacement vers des points spécifiques sur la carte, en planifiant des trajectoires optimisées tout en évitant les obstacles détectés.

## Choix des Composants et Matériels Électroniques ⚙️🔌

Le robot utilise une combinaison spécifique de matériels électroniques pour assurer ses fonctionnalités autonomes.

### Arduino MEGA 💡

L'Arduino MEGA sert de cerveau central pour contrôler les composants du robot, y compris les moteurs, le Lidar et la communication sans fil. J'ai testé ce robot avec un Arduino Uno, mais cela n'a pas fonctionné en raison d'interférences des signaux. L'Arduino Uno ne dispose que d'un seul port série, utilisé à la fois pour le traitement des données du microcontrôleur et pour la communication série. Même en utilisant la bibliothèque SoftwareSerial, cela ne fonctionne pas, car son taux de transmission maximal est de 38400 bauds, alors que mon Lidar LD06 nécessite un taux de transmission de 230400 bauds.

### Moteur Pas à Pas 🔄

Les moteurs pas à pas assurent le mouvement précis des roues du robot, contrôlés pour avancer, reculer, et tourner avec précision.

### Lidar LD06 🌐

Le Lidar LD06 utilise des faisceaux laser pour détecter les distances et les angles des obstacles, essentiel pour la perception de l'environnement.

### Moteur Driver A4988 🚗

Le Driver A4988 contrôle les moteurs pas à pas, calculant les pas nécessaires pour le déplacement précis du robot.

### ESP32 pour Connexion sans Fil 📶

ESP32 permet une communication sans fil via Wi-Fi, connectant le robot au serveur pour des mises à jour en temps réel et une gestion à distance.

### Intégration de l'Alimentation Externe 🔋

Une alimentation externe de 12 volts est intégrée pour alimenter efficacement tous les composants du robot, réduisant ainsi le nombre de câbles et améliorant la portabilité.

## Programmation et Logiciels Utilisés 💻📊

La programmation est essentielle pour coordonner les actions du robot et interpréter les données des capteurs pour une navigation autonome.

### Programmation sur Arduino 🖥️

Arduino est utilisé pour programmer les comportements du robot, y compris la gestion des moteurs, la lecture des données Lidar et la planification des trajectoires.

### Utilisation de Python pour la Visualisation 🐍📊

Python est utilisé pour visualiser graphiquement les données du robot, montrant la trajectoire en temps réel et les points intermédiaires sur la carte 2D construite par le robot.

### Communication avec MATLAB pour le Traitement des Données Lidar 📊🌐

MATLAB est utilisé pour traiter les données brutes du Lidar, en optimisant la détection des obstacles et en améliorant la précision de la cartographie.

## Conclusion 🎓🚀

Ce projet de fin d'année a été une aventure passionnante dans le domaine de la robotique mobile autonome. Il a permis d'explorer et de maîtriser les technologies de pointe tout en développant des compétences essentielles pour une carrière dans l'ingénierie et la robotique.

### Réalisations et Perspectives Futures 🌟

Les réalisations comprennent la conception d'un robot fonctionnel capable de naviguer, cartographier et éviter les obstacles de manière autonome. Les perspectives futures incluent l'amélioration des algorithmes de navigation et l'intégration de capteurs avancés pour une perception améliorée de l'environnement et surtout la réalisation de mon [projet 4]([lien_vers_votre_projet_github](https://github.com/FarajDEV/RobotMobileLivraisonAutonome-InterfaceWeb)) en utilisant ce robot.


### Expériences Acquises et Compétences Développées 🧠🌱

Ce projet a renforcé les compétences en ingénierie mécanique, électronique et logicielle, tout en cultivant une approche collaborative du travail d'équipe et de la résolution de problèmes complexes.

#LabVIEW #Mécatronique #RobotAutonome #Arduino #Python #Matlab #ConversionChiffres #InterfaceUtilisateur #SécuritéInformatique #DéveloppementLogiciel #InnovationTechnologique #IoT #Ingénierie #ProjetÉtudiant #CollaborationTechnique #FutureTech #RobotiqueAvancée #VisionParOrdinateur #TechnologieDePointe #STEM #DéveloppementDeCompétences



![](https://i.imgur.com/waxVImv.png)
