# 📊 Flujo de Funcionamiento - Particle Filter

## 🏗️ Arquitectura General

```
┌─────────────────────────────────────────────────────────────────┐
│                   ROS 2 PARTICLE FILTER NODE                    │
│              (particle_filter_node.py)                           │
└─────────────────────────────────────────────────────────────────┘
                              │
                ┌─────────────┼─────────────┐
                │             │             │
        ┌───────▼──────┐ ┌───▼─────┐ ┌────▼────┐
        │ /scan topic  │ │ /odom   │ │ Timer   │
        │ (LIDAR)      │ │ (motion)│ │ 2.0 Hz  │
        └───────┬──────┘ └───┬─────┘ └────┬────┘
                │             │            │
        ┌───────▼──┐   ┌──────▼─┐   ┌─────▼────┐
        │ Callback │   │Callback│   │ Callback │
        │ scan_    │   │odom_   │   │ timer_   │
        │ saving   │   │saving  │   │callback  │
        └───────┬──┘   └──┬─────┘   └─────┬────┘
                │         │               │
        ┌───────▼─────────▼───────────────▼──────┐
        │  GUARDA DATOS EN HISTORIAL/BUFFER      │
        │  - _scan_last_measures (última scan)   │
        │  - _odometry_estimate_list (buffer)    │
        └────────────────┬──────────────────────┘
                         │
                ┌────────▼────────┐
                │ _timer_callback │  ◄─ PUNTO PRINCIPAL
                │ (cada 2 seg)    │
                └────────┬────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
    ┌───▼────────────┐ ┌─▼──────────────┐ │
    │ Para CADA      │ │ Si hay medidas │ │
    │ velocidad en   │ │ (una vez):     │ │
    │ odometry_list: │ │ MEASUREMENT    │ │
    │                │ │ STEP           │ │
    │ MOTION STEP    │ │                │ │
    └────┬───────────┘ └─┬───────────────┘ │
         │                │                 │
         │            ┌───▼─────────────┐  │
         └──────┬─────┤ compute_pose()  │  │
                │     └───┬─────────────┘  │
                │         │                │
                └─────────┴────────┬───────┘
                                   │
                        ┌──────────▼─────────┐
                        │ Publica /pose      │
                        │ (estimada)         │
                        └────────────────────┘
```

---

## 🔄 BUCLE PRINCIPAL: _timer_callback()

Ejecutada **cada 2 segundos** (timer 2.0 Hz)

```python
_timer_callback()
    │
    ├─► Publica STOP = True
    │
    ├─► PARA CADA (v, w) EN _odometry_estimate_list:
    │       └─► _execute_motion_step(v, w)
    │           (puede haber 0 a N velocidades acumuladas)
    │
    ├─── SI _scan_last_measures tiene datos:
    │       └─► _execute_measurement_step(_scan_last_measures)
    │
    ├─► compute_pose()  [calcula pose estimada con CLUSTERING]
    │
    ├─► Publica /pose
    │
    └─► Publica STOP = False
```

---

## 🏃 MOTION STEP: _execute_motion_step(v, w)

**Flujo de UNA ejecución de movimiento**

```python
_execute_motion_step(z_v, z_w)
    │
    └─► particle_filter.move(z_v, z_w)
        │
        └─► PARA CADA PARTÍCULA EN self._particles:
            │
            ├─► Genera ruido gaussiano para v y w
            │   └─ noise_v ~ N(0, sigma_v)
            │   └─ noise_w ~ N(0, sigma_w)
            │
            ├─► Actualiza posición (x, y, theta) con el ruido
            │   └─ x += cos(theta) * (v + noise_v) * dt
            │   └─ y += sin(theta) * (v + noise_v) * dt
            │   └─ theta += (w + noise_w) * dt
            │
            └─► Verifica colisiones con el mapa
                └─ Si intersecta pared: ajusta posición
```

**Resultado**: Todas las partículas se mueven **en paralelo** según la misma velocidad.

---

## 📊 MEASUREMENT STEP: _execute_measurement_step(z_scan)

**Flujo de UNA ejecución de medición**

```python
_execute_measurement_step(z_scan)
    │
    ├─► particle_filter.resample(z_scan)
    │   │
    │   └─► PARA CADA PARTÍCULA:
    │       │
    │       └─► _measurement_probability(z_scan, particle)
    │           │
    │           ├─► _sense(particle_pose) 
    │           │   │   [Simula los rayos del LIDAR para esta partícula]
    │           │   │
    │           │   ├─► _lidar_rays(pose, indices=[0, 30, 60, 90, 120, 150, 180, 210])
    │           │   │    └─ Crea 8 rayos evenly spaced
    │           │   │
    │           │   └─► PARA CADA RAYO:
    │           │       └─ Calcula distancia a obstáculo (map.check_collision)
    │           │       
    │           │   RETORNA: predicted_measurements (lista de 8 distancias)
    │           │
    │           ├─► PARA CADA medida observada (8 rayos subsampled):
    │           │   │
    │           │   ├─ measurement = z_scan[ray_index]
    │           │   ├─ predicted = predicted_measurements[i]
    │           │   │
    │           │   ├─► _gaussian(measurement, predicted, sigma_z)
    │           │   │    └─ Calcula P(medida | partícula)
    │           │   │
    │           │   └─ probability *= P(medida | partícula)
    │           │
    │           └─► RETORNA: probability (probabilidad final)
    │
    │   [Al salir del bucle tenemos N probabilidades (una por partícula)]
    │
    │   ├─► Normaliza probabilidades (suma = 1)
    │   │
    │   └─► Resampling estratificado (stratified sampling):
    │       └─ Muestrea partículas proporcionales a su probabilidad
    │           └─ Partículas con alta probabilidad → más copias
    │           └─ Partículas con baja probabilidad → pueden desaparecer
    │
    ├─► particle_filter.compute_pose()
    │   │
    │   ├─► Proyecta partículas a 3D (x,y, cos(θ), sin(θ))
    │   │
    │   └─► DBSCAN clustering (eps=0.2, min_samples=5)
    │       │
    │       ├─ SI n_clusters == 1:
    │       │   └─► localized = True
    │       │   └─► Calcula centroide del cluster
    │       │   └─► RETORNA: pose estimada (x_mean, y_mean, theta_mean)
    │       │   └─► Reduce partículas a 50 (tracking mode)
    │       │
    │       ├─ ELIF n_clusters > 1:
    │       │   └─► localized = False
    │       │   └─► Aumenta partículas según clusters
    │       │       └─ particle_count = max(50 * n_clusters, 50)
    │       │
    │       └─ ELSE (n_clusters == 0):
    │           └─► localized = False
    │
    └─► RETORNA: (x_h, y_h, theta_h) o (inf, inf, inf)
```

---

## 📈 MATRICES DE LLAMADAS POR CONTEXTO

### **Contexto: MOVIMIENTO (en bucle para cada v,w)**

```
_timer_callback()
    │
    FOR z_v, z_w IN _odometry_estimate_list:  ◄─ BUCLE 1 (velocidades acumuladas)
        │
        └─► _execute_motion_step(z_v, z_w)
            │
            └─► particle_filter.move(z_v, z_w)
                │
                FOR particle IN particles:  ◄─ BUCLE 2 (N partículas)
                    ├─ Actualiza x, y, theta
                    └─ Verifica colisión con map
```

**Bucles anidados:**
- Bucle externo: 0 a N velocidades por iteración del timer
- Bucle interno: siempre N partículas (típicamente 50-1000)

### **Contexto: MEDICIÓN (una vez)**

```
_timer_callback()
    │
    IF _scan_last_measures is not None:
        │
        └─► _execute_measurement_step(_scan_last_measures)
            │
            └─► particle_filter.resample(z_scan)
                │
                FOR particle IN particles:  ◄─ BUCLE 1 (N partículas)
                    │
                    └─► _measurement_probability(z_scan, particle)
                        │
                        ├─► _sense(particle_pose)
                        │   │
                        │   ├─► _lidar_rays(pose, indices=[8 rayos])
                        │   │
                        │   FOR ray IN rays:  ◄─ BUCLE 2 (8 rayos del LIDAR)
                        │       └─ map.check_collision(ray_segment)
                        │
                        └─► FOR measurement, predicted_meas:  ◄─ BUCLE 3 (8 medidas)
                            └─► _gaussian(mu, sigma, x)
```

**Bucles anidados:**
1. Bucle externo: N partículas
2. Bucle interno 1: 8 rayos del LIDAR (por partícula)
3. Bucle interno 2: 8 medidas (comparación teoría vs realidad)

---

## 🎯 PUNTOS CLAVE DE PARALELIZACIÓN

| Operación | ¿Se puede paralelizar? | Dónde | Por qué |
|-----------|--------|-------|---------|
| **move()** | ✅ Sí | Bucle de partículas | Cada partícula es independiente |
| **_measurement_probability()** | ✅ Sí | Bucle de partículas | Cada partícula es independiente |
| **_sense()** | ✅ Limitado | Bucle de rayos | Rayos son independientes |
| **_gaussian()** | ✅ Sí | Bucle de medidas | Cálculos independientes |
| **resample()** | ❌ No | Operación global | Necesita todas las probabilidades primero |
| **compute_pose()** | ❌ Parcial | Después del resample | DBSCAN no es trivial de paralelizar |
| **Timer callback** | ❌ No | Después de cada tick | Sincronización ROS2 |

---

## ⏱️ LÍNEA DE TIEMPO TÍPICA (1 iteración)

```
t=0s:   ROS2 recibe /odometry (v1=0.5, w1=0.1)
        ├─ _callback_odometry_saving_history()
        └─ Se agrega (0.5, 0.1) a _odometry_estimate_list

t=0.1s: ROS2 recibe /scan (240 rayos)
        ├─ _callback_scan_saving_history()
        └─ Se guarda en _scan_last_measures

t=0.2s: Llega otra /odometry (v2=0.5, w2=0.1)
        └─ Se agrega (0.5, 0.1) a _odometry_estimate_list

        [Lista actual: [(0.5, 0.1), (0.5, 0.1)]]

t=2.0s: ⏰ TIMER TICK (_timer_callback)
        │
        ├─► FOR (0.5, 0.1) // iteración 1
        │   └─► move(v=0.5, w=0.1) // actualiza N partículas
        │   └─ Tiempo: ~5-10ms (N=1000)
        │
        ├─► FOR (0.5, 0.1) // iteración 2
        │   └─► move(v=0.5, w=0.1) // actualiza N partículas
        │   └─ Tiempo: ~5-10ms
        │
        ├─► resample(z_scan)
        │   └─► Calcula P para cada partícula
        │   └─ Tiempo: ~50-100ms
        │
        ├─► compute_pose()
        │   └─► DBSCAN clustering
        │   └─ Tiempo: ~10-20ms
        │
        └─► Publica /pose
            └─ Tiempo: ~1ms

        Total: ~70-140ms por ciclo (2s disponibles)
```

---

## 📝 RESUMEN DE FUNCIONES CLAVE

### **ParticleFilter (particle_filter.py)**

| Función | Entrada | Funcionalidad | Bucles | Llamado por |
|---------|---------|---------------|--------|------------|
| `__init__()` | Parámetros | Inicializa PF | - | Node startup |
| `move(v, w)` | velocidades | Actualiza posiciones | ∀ partícula | _execute_motion_step |
| `resample(z_scan)` | medidas LIDAR | Resampling + cálculo P | ∀ partícula, ∀ rayo | _execute_measurement_step |
| `compute_pose()` | - | Clustering → pose | ∀ partícula | _timer_callback |
| `_measurement_probability()` | medidas, partícula | Calcula P(medidas\|partícula) | ∀ medida | resample (∀ partícula) |
| `_sense(pose)` | pose partícula | Simula LIDAR | ∀ rayo | _measurement_probability |
| `_gaussian()` | mu, sigma, x | Función gaussiana | - | _measurement_probability |
| `_lidar_rays()` | pose, indices | Genera rayos LIDAR | ∀ rayo | _sense |
| `_init_particles()` | count, modo, inicial | Crea partículas válidas | ∀ partícula | __init__ |
| `plot()` / `show()` | - | Visualización | - | opcional |

### **ParticleFilterNode (particle_filter_node.py)**

| Función | Disparo | Funcionalidad | Bucles |
|---------|---------|---------------|--------|
| `_timer_callback()` | Timer ROS2 (2Hz) | Orquesta PF completo | ∀ velocidad, si medida |
| `_execute_motion_step()` | _timer_callback | Ejecuta move() | 1 step |
| `_execute_measurement_step()` | _timer_callback | Ejecuta resample() + compute_pose() | 1 step |
| `_callback_scan_saving_history()` | /scan topic | Guarda última medida LIDAR | - |
| `_callback_odometry_saving_history()` | /odom topic | Guarda velocidades (buffer) | - |
| `_publish_pose_estimate()` | _timer_callback | Publica /pose | - |

---

## 🔗 DEPENDENCIA DE DATOS

```
Iteration N:
├─ INPUT: /scan y /odom (múltiples mensajes en 2 segundos)
│
├─ PROCESAMIENTO:
│   ├─ move(v₁, w₁)  // Partículas n → n'
│   ├─ move(v₂, w₂)  // Partículas n' → n''
│   │
│   ├─ resample(z_scan)
│   │   └─ Necesita: partículas n'' (del move)
│   │   └─ Calcula: probabilidades de cada partícula
│   │   └─ Salida: partículas remuestreadas
│   │
│   └─ compute_pose()
│       └─ Necesita: partículas remuestreadas
│       └─ Salida: pose estimada + _particle_count ajustado
│
└─ OUTPUT: /pose publicada
```

---

## 🧮 COMPLEJIDAD COMPUTACIONAL

```
Por iteración del timer:

Motion steps: O(M × N)
  M = número de movimientos acumulados
  N = número de partículas

Measurement step: O(N × R × M)
  N = número de partículas
  R = 8 rayos del LIDAR
  M = número de medidas (240)

Resampling: O(N log N) o O(N)

Clustering: O(N d²) DBSCAN
  N = partículas
  d = dimensión (3 o 4)

Total: Dominado por measurement step O(N × 240) ≈ O(N × 240)
```

