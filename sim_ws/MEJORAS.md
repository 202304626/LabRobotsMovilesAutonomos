# Mejoras sugeridas (Python, hardcore)

Orden recomendado: 1) Wall follower, 2) Pure pursuit, 3) Particle filter, 4) PRM, 5) Bringup/QoS, 6) Parámetros/telemetría.

## Wall follower (`amr_control/wall_follower.py`, `_node.py`)
Robustez (por qué)
- Filtrar `z_scan` (NaN/inf, mediana por sector, validar longitud): evita decisiones con datos corruptos y reduce ruido que dispara giros falsos.
- Histeresis + permanencia de estado: elimina zigzagueo en umbrales; obliga a completar un giro antes de cambiar de sentido.
- Selección de pared estable: fija pared al inicio y solo cambia tras pérdida prolongada; evita alternar por diferencias mínimas.
- Clamp de comandos: respeta límites físicos; ajustar `v` con `|w|` evita saturar motores al girar.
- Watchdog de latencia: mejor parar que comandar con datos atrasados.

Velocidad (por qué)
- `np.ndarray` + índices precomputados + `np.nanmin`: menos copias y CPU por ciclo.
- Logs solo en eventos: no bloquea el lazo.
- Umbrales parametrizados: tuning rápido sin tocar código.

Implementación concreta
- Preprocesar LiDAR: convertir a array, limpiar NaN/inf, mediana 3–5 muestras por sector; si falta sector → (0,0).
- Histeresis: `front_in < front_out`; contador mínimo de ciclos en `Turn_*` antes de volver a `Front`.
- Clamp: `w = clip(w, -w_max, w_max)`; `v = clip(v, 0, v_max - |w|*TRACK/2)` antes de devolver/publicar.
- Watchdog: en callback, si `now - scan.stamp > 0.2–0.5 s`, no computar y (opcional) publicar stop.

## Pure pursuit (`amr_control/pure_pursuit.py`, `_node.py`)
Robustez (por qué)
- Normalizar `alpha` y tratar eps: evita divisiones por cero y picos de `w` cuando el objetivo está alineado.
- Lookahead dinámico: estabilidad a alta velocidad y precisión a baja; reduce overshoot.
- Freno final con tolerancia: evita orbitar el último punto; detiene explícitamente.
- Limpieza de ruta entrante: quita NaN/duplicados que causan saltos de ángulo.

Velocidad (por qué)
- Ruta como `np.ndarray` y KDTree opcional: reduce coste de “closest point” en rutas largas en cada ciclo.
- Publicación condicionada (deadband): menos tráfico y CPU en la pipeline.
- Logs throttleados: menos impacto en RT.

Implementación concreta
- Normalizar `alpha` a [-pi, pi]; si `l < eps` o `|sin(alpha)| < eps`, (0,0) o `w=0`.
- Lookahead: `Ld = clip(L0 + k*v, Ld_min, Ld_max)`; parámetros configurables.
- Freno final: si dist al último punto < tol, publicar stop y latchear fin.
- Setter `path`: limpia NaN/duplicados, aplica delta mínima, almacena `np.ndarray`; opcional KDTree.
- Deadband en publicación: si |Δv|<dv_min y |Δw|<dw_min, no publicar salvo cada N ciclos.

## Particle filter (`amr_localization/particle_filter.py`, `_node.py`)
Robustez (por qué)
- Log-pesos + `logsumexp`: evita underflow y mantiene discriminación entre partículas.
- Resample adaptativo con `N_eff`: solo resamplear cuando la nube se degenera; ahorra CPU y evita empobrecimiento.
- Clamp de mediciones y NaN→range_min: evita pesos cero/infinito por lecturas fuera de rango.
- Colisión con retroceso y ruido angular: evita que partículas atraviesen paredes y se queden pegadas.
- Ruta del mapa única: elimina riesgos de path incorrecto o fallo al cargar.
- DBSCAN con casos borde: 0 clusters → seguir global; >1 → subir N; 1 → bajar a tracking.

Velocidad (por qué)
- Vectorizar `move` y probabilidad: mayor ahorro de CPU; bucles Python son el cuello de botella.
- Submuestreo de rayos configurable: menos coste en probabilidad con información suficiente.
- Log-likelihood en vez de producto: más estable y rápido de normalizar.
- `steps_btw_sense_updates` dinámico: menos sense/cluster cuando ya estás localizado o vas lento.
- Logs throttleados: menos impacto en RT.

Implementación concreta
- `move`: precomputar `cos/sin(theta)`; actualizar x,y,theta en vector; aplicar colisión/retroceso por partícula.
- `_measurement_probability`: clamp mediciones, log-pesos, normalizar con `logsumexp`; epsilon para evitar log(0).
- `N_eff`: `1/sum(w^2)`; si < tau*N → resample; si no, saltar; ajustar N tras DBSCAN.
- Submuestreo rayos: índices fijos y configurables.
- Unificar construcción de `map_path` (una estrategia) y validar existencia.

## PRM (`amr_planning/prm.py`)
Robustez (por qué)
- Validar start/goal antes de planificar: falla rápido si es inválido.
- `_reconstruct_path` con error claro: depuración más rápida en grafos desconectados.
- Evitar doble inserción de vecinos: grafo limpio y sin costes extra.

Velocidad (por qué)
- `_connect_nodes` con `cKDTree`: reduce conexión de O(n²) a O(n log n).
- Cachear roadmap (pickle + hash de mapa/parámetros): evita recalcular roadmap entre ejecuciones.
- Limitar grid/nodos a zona libre dilatada: menos nodos, mismo espacio útil.
- Sin plots en runtime: evita bloqueos.

Implementación concreta
- KDTree de radio para vecinos; sets → listas al final.
- Guardar/cargar roadmap con hash simple de mapa y parámetros.
- Validar start/goal con `Map.contains`; mensajes específicos.

## Bringup / QoS / sincronización
Robustez (por qué)
- QoS LiDAR: `SensorDataQoS` (BestEffort, depth corto) para minimizar latencia; odom reliable, depth corto.
- `ApproximateTimeSynchronizer`: `queue_size` pequeño (5–10), `slop` 0.2–0.5 s; menos lag.
- `frame_id` coherente: evita errores de TF downstream.
- Watchdog de latencia: si datos atrasados, saltar ciclo y opcional stop.

Velocidad (por qué)
- Slop corto y colas pequeñas reducen el retardo efectivo.
- Menos logs en callbacks de control.

Implementación concreta
- Ajustar QoS y sincronizador en nodos; chequear latencia antes de procesar.
- Parar robot si falta LiDAR/odom > T.

## Parámetros base (punto de arranque)
- Wall follower: `Kp=4`, `Kd=5`; `front_distance_threshold=0.27`; `expected_turning_distance=threshold*sqrt(2)`; `w_max≈1.5 rad/s` (ajustar a hardware).
- Pure pursuit: `Ld=0.25` a `v=0.1–0.15`; `Ld = 0.2 + 1.0*v` con límites [0.2, 0.5]; tol fin de ruta 0.1 m; eps ángulo 1e-3.
- Particle filter: `sigma_z=0.07`; `sigma_v=0.05`; `sigma_w=0.08`; DBSCAN `eps=0.2`, `min_samples=6`; `N_eff` umbral 0.5*N; subconjunto rayos=8 (configurable).
