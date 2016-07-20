```
```
#
``` FootPreassureSensor
```
Intro to component here

El componente foot pressure devuelve la presión leida por un sensor colocado en los extremos de las patas de la araña, usados para conocer qué pierna apoya con mayor fuerza en cada pata y emplearlo para realizar movimientos

Métodos de la interfaz foot pressure

 interface FootPreassureSensor
    {
        Buffer readSensors();
	int readSensor(string name);
    };

readSensors() devuelve los valores de todos los sensores de presión en un string.
readsensor(name) devuelve el valor del sensor que solicitamos mediante la variable name.

Los nombres de las patas son:
p1 (delantera izquierda)
p2 (delantera derecha)
p3 (media izquierda)
p4 (media derecha)
p5 (trasera izquierda)
p6 (trasera derecha)



## Configuration parameters
As any other component,
``` *FootPreassureSensor* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <FootPreassureSensor 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```FootPreassureSensor ```

    --Ice.Config=config
