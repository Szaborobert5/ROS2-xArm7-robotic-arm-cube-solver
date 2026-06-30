# Uputstvo

Pre nego sto se kompajlira projekat potrebno u terminalu u `<projekat>/src` 
putanji pokrenuti:

```sh
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Kako bi se sve potrebne biblioteke instalirale.

Nakon toga:

```sh
colcon build
```

Proces ce potrajati od 3-10min u zavisnosti od racunara. Budite strpljivi.

Da proverite da sve radi, nakon sto se prethodna komanda izvrsila:

```sh
source install/setup.bash

ros2 launch project_2026 task.launch.py
```

Ovo treba da pokrene:
- Gazbo simulator
- RViz za vizualizaciju

