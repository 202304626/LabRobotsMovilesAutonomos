# Migración a C++: qué y por qué

## Qué migrar primero
1) **Particle filter (núcleo)**
   - Migrar: `move`, `_measurement_probability`, resample; sustituir DBSCAN por clustering en C++ o PCL/FLANN.
   - Por qué: mayor consumo de CPU; C++ + Eigen reduce tiempos y permite más partículas o menor latencia.

2) **PRM (construcción/consulta)**
   - Migrar: generación de nodos, conexión con vecinos en radio (KDTree), A*; opcional smoothing. Cache binario del roadmap.
   - Por qué: la construcción es costosa en Python; C++ con nanoflann/kdtree acelera y soporta mapas más densos.

3) **Control lazo corto**
   - Wall follower y pure pursuit (cálculo de `v,w`).
   - Por qué: eliminar jitter/GC de Python en lazo >20 Hz; menor latencia de callback en rclcpp.

4) **Preprocesado LiDAR**
   - Filtrado rápido (NaN clamp, mediana, downsample) en C++ integrado en nodos críticos.
   - Por qué: reduce overhead Python y estabiliza control.

## Qué no urge migrar
- Bringup, orquestación y configuración de parámetros: overhead bajo; mantener en Python facilita iteraciones.
- Telemetría/logs: más ágil en Python; no crítico para rendimiento.

## Migrar todo vs migrar partes
- Todo a C++: máximo determinismo, pero más mantenimiento y menos agilidad.
- Partes críticas: equilibrio recomendado. Migrar PF núcleo + PRM + callbacks de control si se requieren frecuencias altas.
- Funciones aisladas: exponer kernels C++ (pybind11) y mantener nodo Python. Útil para PF/PRM: reduces CPU en hot paths sin reescribir nodos.

## Recomendación por nodo
- `particle_filter_node`: migrar completo a rclcpp con Eigen y KDTree; o nodo Python llamando a lib C++ para `move/prob/resample` vía pybind11.
- `prm_node` (si online): mover construcción/planificación a C++ con nanoflann; exponer servicio/acción en C++.
- `wall_follower_node`: opcional migrar a C++ si necesitas >20–30 Hz con jitter bajo; lógica simple.
- `pure_pursuit_node`: migrar si subes frecuencia o en hardware modesto.
- Preprocesado LiDAR: integrar en nodos C++ anteriores.

## Librerías y stack sugeridos
- Eigen para álgebra; nanoflann (o PCL) para KDTree/vecinos; rclcpp para nodos; tf2 para transforms.
- Opcional: range-v3 para claridad; mantener dependencias mínimas.

## Plan de migración
1) PF: portar `move/measurement/resample` a C++; medir tiempos vs Python; decidir migrar nodo completo a rclcpp.
2) PRM: portar generación+conexión+planificación; añadir cache binario; exponer servicio/acción.
3) Control: portar callbacks de `cmd_vel` (wall follower/pure pursuit) a rclcpp si la latencia actual no es estable; reutilizar lógica existente.
4) Percepción: añadir filtro LiDAR C++ (NaN clamp, mediana, downsample) en nodos de control/localización.
5) Mantener bringup/orquestación en Python para agilidad.

## Criterios de decisión
- Perfilado: si el lazo >50% de tiempo en PF/PRM Python, migrar ese kernel a C++.
- Frecuencia objetivo: si necesitas >20–30 Hz con jitter bajo, mover control y preprocesado a C++.
- Mantenibilidad: si el equipo es reducido, priorizar kernels (pybind11) antes que reescribir nodos completos.
