# ASR-0-AvoidObstacle
Práctica 0 - Arquitecturas Software para Robots 2023

Crea un comportamiento autónomo de un robot usando una máquina de estado finito (FSM) para evitar obstáculos:
1. El robot empieza parado, y comienza su comportamiento cuando se pulsa un botón del robot.
2. El robot avanza hasta encontrar un obstáculo a menos de un metro enfrente de él.
3. Cuando encuentra un obstáculo, el robot gira 90 grados, y realiza un movimiento de arco para sobrepasarlo.
4. Si mientras está haciendo el arco, se encuentra un nuevo obstáculo, vuelve a hacer lo mismo del punto 3.

![asr_practica_0](https://user-images.githubusercontent.com/3810011/217230998-a162f2e1-cf50-4e26-9155-53ca73e99f86.png)

El robot debe funcionar en el robot real Kobuki.

Puntuación (sobre 10):

* +8 correcto funcionamiento en el robot real.
* +2 Readme.md bien documentado con videos.
* -3 Warnings o que no pase los tests.
* +1 Setup de CI/CD

Imágenes:

# Versión 1: Avoid_obstacle

## Contenido multimedia

[VÍDEO]

## Máquina de estados implementada para este modelo

[IMAGEN]

## Características de uso e implementación



# Versión 2: Avoid_obstacle_advanced

## Contenido multimedia

[VÍDEO]

## Máquina de estados implementada para este modelo

[IMAGEN]

## Características de uso e implementación

A diferencia del modelo explicado previamente, está implementación está basada en un nodo cuya responsabilidad es la de analizar y gestionar el comportamiento del robot Kobuki basándose en la información que va tomando del láser. Además, la máquina de estados usada para este modelo se basa en una máquina de estados finitos bidimensional probabilística, que es capaz de mapear el entorno e ir esquivando los obstáculos que se encuentra en el camino.

La implementación de este modelo está principalmente compuesta por cuatro grandes funciones, una por cada algoritmo implementado. A continuación se detalla una breve explicación de cada uno de ellos:

* Sectorización de valores (sectorize()): Divide un conjunto de valores float (asociados al láser) en un conjunto de sectores.

* Búsqueda de obstáculos (obstacleAnalyze()): Calcula el peligro asociado a una determinada sectorización acorde a los valores obtenidos anteriormente, devolviendo un valor booleano en caso de que no haya peligro (0) o se detecte algún obstáculo (1).

* Distancia al obstáculo(calculate_side_to_avoid_obstacle()): Determina cuál es el mejor lado para evitar un obstáculo, en otras palabras, decide qué sector de los valores obtenidos por el láser le debe dar más prioridad en caso de que se detecte un obstáculo.

* Valores de intensidad (setIntensities()): Traduce uno de los sectores de valores obtenidos anteriormente a un mapa de calor para después graficarlos.
