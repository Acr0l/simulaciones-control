# Simulaciones de Control Automático en MuJoCo

Este repositorio contiene una serie de entornos simulados en MuJoCo, diseñados para ayudar a los estudiantes de control automático a experimentar y probar diferentes algoritmos y técnicas en un espacio digital. Los entornos permiten la simulación de sistemas dinámicos y de control que son difíciles o costosos de recrear en el mundo real.

## ¿Qué es MuJoCo?

MuJoCo (Multi-Joint dynamics with Contact) es un motor de simulación física avanzado que permite simular sistemas mecánicos con múltiples cuerpos, contactos y articulaciones. Es ampliamente utilizado en robótica, biomecánica, y en el diseño de sistemas de control automático.

### Características principales de MuJoCo

- **Simulación precisa y rápida**: MuJoCo está diseñado para realizar cálculos rápidos de dinámicas en tiempo real.
- **Flexibilidad en la definición de modelos**: Puedes crear modelos personalizados, definir restricciones, fricción, contacto, y más.
- **Integración con Python**: Se puede controlar la simulación mediante scripts en Python, lo que permite automatizar experimentos y realizar análisis avanzados.

## Requisitos Previos

Para utilizar este repositorio, necesitarás:

- Python 3.x
- [MuJoCo](http://www.mujoco.org/) instalado
- La librería `mujoco`

Instala las dependencias ejecutando:

```bash
pip install mujoco
```

## Cómo Usar el Repositorio

Este repositorio contiene una serie de modelos listos para ser simulados. Puedes cargar y ejecutar las simulaciones utilizando Python. A continuación, se muestra un ejemplo básico de cómo cargar un modelo y controlar la simulación:

### Ejemplo de Código

```python
import mujoco
import os

# Cargar el modelo MuJoCo
model_path = "ruta/al/modelo.xml"  # Cambia esto a la ubicación del archivo XML
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model,data) as viewer:
    while viewer.is_running():

        # Código de control

        mujoco.mj_step(model, data)

        viewer.sync()
```

Este código inicializa un modelo en MuJoCo, crea una simulación y ejecuta un bucle en el cual puedes observar la evolución del sistema. También puedes modificar los actuadores para controlar los componentes del modelo en tiempo real.

## Estructura del Repositorio

- `/models`: Contiene los modelos MuJoCo en formato XML que representan diferentes sistemas de control automático.
- `/scripts`: Contiene scripts de Python para cargar y controlar las simulaciones.
- `/docs`: Documentación y guías sobre los experimentos incluidos.

## Contribuciones

Este repositorio está en constante desarrollo. Si tienes alguna sugerencia, mejora o deseas contribuir, ¡eres bienvenido a hacerlo! Puedes enviar un pull request o abrir un issue con tus ideas.

## Licencia

Este proyecto está licenciado bajo la [MIT License](LICENSE).

---
