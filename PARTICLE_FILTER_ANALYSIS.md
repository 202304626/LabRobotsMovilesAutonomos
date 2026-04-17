# ANÁLISIS DETALLADO DE PARTICLE FILTER - FLUJO DE FUNCIONES Y BUCLES

## 1. ESTRUCTURA GENERAL DEL SISTEMA

```
ROS 2 Lifecycle
    ↓
ParticleFilterNode.__init__()
    ↓ (Declaración de parámetros)
ParticleFilterNode.on_configure()
    ├─ Carga parámetros ROS 2
    ├─ Crea ParticleFilter() 
    │  └─ Inicializa 1000 partículas aleatorias
    ├─ Crea subscribers a /odometry y /scan
    └─ Registra callback sincronizado
        ↓
    CICLO PRINCIPAL (cuando llegan datos sincronizados)
        ↓
    ParticleFilterNode._compute_pose_callback()
        ├─ Extrae velocidades de /odometry
        ├─ Extrae medidas de /scan
        ├─ Llama a _execute_motion_step()
        ├─ Llama a _execute_measurement_step()
        └─ Publica /pose estimada
```

---

## 2. ESTRUCTURA DE CLASES

```
┌──────────────────────────────────────────────────────────┐
│              ParticleFilterNode (LifecycleNode)          │
├──────────────────────────────────────────────────────────┤
│                                                           │
│ INIT:                                                    │
│ • self.pf_instance = None                               │
│ • self._steps = 0                                        │
│ • self._localized = False                                │
│                                                           │
│ LIFECYCLE METHODS:                                       │
│ • on_configure()     ← Crea ParticleFilter               │
│ • on_activate()      ← Activa node                       │
│                                                           │
│ CALLBACKS:                                               │
│ • _compute_pose_callback(odom_msg, scan_msg)            │
│                                                           │
│ HELPER METHODS:                                          │
│ • _execute_motion_step(z_v, z_w)                        │
│ • _execute_measurement_step(z_scan)                     │
│ • _publish_pose_estimate(x, y, theta)                   │
│                                                           │
└──────────────────────────────────────────────────────────┘
                           ↓
            Crea y utiliza
                           ↓
┌──────────────────────────────────────────────────────────┐
│              ParticleFilter (Algoritmo)                  │
├──────────────────────────────────────────────────────────┤
│                                                           │
│ INIT:                                                    │
│ • self._particles = np.array((1000, 3))  ← Partículas   │
│ • self._map = Map(...)                   ← Mapa         │
│ • self._iteration = 0                                    │
│                                                           │
│ MAIN ALGORITHM METHODS:                                  │
│ • move(v, w)              ← Predicción (motion step)     │
│ • resample(measurements)  ← Actualización (resampling)  │
│ • compute_pose()          ← Estimación de pose          │
│                                                           │
│ HELPER METHODS:                                          │
│ • _init_particles(...)    ← Inicializa partículas       │
│ • _sense(pose)            ← Simula sensor               │
│ • _measurement_probability(measurements, particle)       │
│ • _gaussian(mu, sigma, x) ← Función probabilística      │
│ • _lidar_rays(pose, indices)  ← Simula rayos LiDAR     │
│                                                           │
│ VISUALIZATION:                                           │
│ • plot()                                                 │
│ • show()                                                 │
│                                                           │
└──────────────────────────────────────────────────────────┘
```

---

## 3. FLUJO DE EJECUCIÓN PRINCIPAL - SYNC (sim_ws)

```
═══════════════════════════════════════════════════════════════════════════════
                    CICLO DE ACTUALIZACIÓN COMPLETO
═══════════════════════════════════════════════════════════════════════════════

EVENTO: /odometry y /scan llegan sincronizados
   ↓
   ↓ (ApproximateTimeSynchronizer dispara callback)
   ↓
_compute_pose_callback(odom_msg: Odometry, scan_msg: LaserScan)
   │
   ├─ [EXTRACCIÓN DE DATOS]
   │  ├─ z_v = odom_msg.twist.twist.linear.x        ← Velocidad lineal [m/s]
   │  ├─ z_w = odom_msg.twist.twist.angular.z       ← Velocidad angular [rad/s]
   │  └─ z_scan = list(scan_msg.ranges)             ← 240 medidas LiDAR [m]
   │
   ├─ _execute_motion_step(z_v, z_w)
   │  │  ↓ PASO 1: PREDICCIÓN (MOTION UPDATE)
   │  │
   │  ├─ start_time = perf_counter()
   │  │
   │  ├─ ParticleFilter.move(z_v, z_w)
   │  │  │
   │  │  ├─ self._iteration += 1
   │  │  │
   │  │  └─ ┌─ FOR i IN range(N_particles=1000):  ← BUCLE 1: POR CADA PARTÍCULA
   │  │     │
   │  │     ├─ particle = self._particles[i]  ← Acceso a partícula (x, y, θ)
   │  │     │
   │  │     ├─ ┌─ PREDICCIÓN CINEMÁTICA
   │  │     │  │
   │  │     │  ├─ noise_v = random.gauss(0, sigma_v)    ← Ruido velocidad lineal
   │  │     │  ├─ noise_w = random.gauss(0, sigma_w)    ← Ruido velocidad angular
   │  │     │  │
   │  │     │  ├─ x_vel = cos(θ) * (z_v + noise_v)      ← Velocidad en X
   │  │     │  ├─ y_vel = sin(θ) * (z_v + noise_v)      ← Velocidad en Y
   │  │     │  │
   │  │     │  ├─ Δx = x_vel * dt
   │  │     │  ├─ Δy = y_vel * dt
   │  │     │  ├─ Δθ = (z_w + noise_w) * dt
   │  │     │  │
   │  │     │  ├─ particle[0] += Δx                      ← Update X
   │  │     │  ├─ particle[1] += Δy                      ← Update Y
   │  │     │  ├─ particle[2] = (particle[2] + Δθ) % 2π  ← Update Theta (normalizado)
   │  │     │  └─
   │  │     │
   │  │     ├─ ┌─ VERIFICACIÓN DE COLISIÓN
   │  │     │  │
   │  │     │  ├─ old_pose = (x_old, y_old)
   │  │     │  ├─ new_pose = (particle[0], particle[1])
   │  │     │  │
   │  │     │  ├─ collision_point = self._map.check_collision(
   │  │     │  │      [(old_pose), (new_pose)],
   │  │     │  │      compute_distance=False
   │  │     │  │  )
   │  │     │  │
   │  │     │  └─ [IF collision_detected]:
   │  │     │     ├─ particle[0] = collision_point[0]    ← Posiciona en colisión
   │  │     │     └─ particle[1] = collision_point[1]
   │  │     │
   │  │     └─ [FIN ITERACIÓN]
   │  │
   │  ├─ move_time = perf_counter() - start_time
   │  │
   │  └─ [IF enable_plot]: show("Move", save_figure=True)
   │
   │
   ├─ _execute_measurement_step(z_scan) 
   │  │  ↓ PASO 2: ACTUALIZACIÓN (MEASUREMENT UPDATE)
   │  │
   │  ├─ [IF (self._steps % steps_btw_sense_updates == 0) OR localized]:
   │  │  │  ← SOLO CADA 10 ITERACIONES para ahorrar cálculo
   │  │  │
   │  │  ├─ start_time = perf_counter()
   │  │  │
   │  │  ├─ ParticleFilter.resample(z_scan)
   │  │  │  │
   │  │  │  ├─ probabilities = np.zeros(N_particles=1000)
   │  │  │  │
   │  │  │  └─ ┌─ FOR i IN range(N_particles=1000):  ← BUCLE 2: POR CADA PARTÍCULA
   │  │  │     │
   │  │  │     ├─ particle = self._particles[i]
   │  │  │     │
   │  │  │     ├─ probabilities[i] = _measurement_probability(z_scan, particle)
   │  │  │     │  │
   │  │  │     │  ├─ ┌─ PASO A: Obtener medidas predichas
   │  │  │     │  │  │
   │  │  │     │  │  ├─ z_hat = _sense(particle)
   │  │  │     │  │  │  │
   │  │  │     │  │  │  └─ ┌─ rays = [0, 30, 60, 90, 120, 150, 180, 210]
   │  │  │     │  │  │     │
   │  │  │     │  │  │     ├─ segments = _lidar_rays(particle_pose, rays)
   │  │  │     │  │  │     │  │
   │  │  │     │  │  │     │  └─ ┌─ FOR j IN range(8):  ← BUCLE 3: POR CADA RAYO
   │  │  │     │  │  │     │     │
   │  │  │     │  │  │     │     ├─ ray_angle = 1.5° * j  ← Ángulo del rayo
   │  │  │     │  │  │     │     ├─ x_end, y_end = project ray to sensor_range_max
   │  │  │     │  │  │     │     │
   │  │  │     │  │  │     │     ├─ segment = [(x_sensor, y_sensor), (x_end, y_end)]
   │  │  │     │  │  │     │     └─ segments.append(segment)
   │  │  │     │  │  │     │
   │  │  │     │  │  │     └─ RETURN 8 segmentos de rayos
   │  │  │     │  │  │
   │  │  │     │  │  ├─ ┌─ FOR j IN range(8):  ← BUCLE 3b: CALCULAR DISTANCIA POR RAYO
   │  │  │     │  │  │  │
   │  │  │     │  │  │  ├─ collision_point, distance = self._map.check_collision(
   │  │  │     │  │  │  │      segments[j],
   │  │  │     │  │  │  │      compute_distance=True
   │  │  │     │  │  │  │  )
   │  │  │     │  │  │  │
   │  │  │     │  │  │  └─ z_hat[j] = distance
   │  │  │     │  │  │
   │  │  │     │  │  └─ RETURN z_hat (8 distancias predichas)
   │  │  │     │  │
   │  │  │     │  ├─ ┌─ PASO B: Subsampling de medidas reales
   │  │  │     │  │  │
   │  │  │     │  │  ├─ rays = range(0, 240, 30)  ← [0, 30, 60, 90, 120, 150, 180, 210]
   │  │  │     │  │  │
   │  │  │     │  │  └─ ┌─ FOR j IN range(8):  ← BUCLE 3c: EXTRAER MEDIDAS
   │  │  │     │  │     │
   │  │  │     │  │     ├─ measurement[j] = z_scan[rays[j]]  ← Medida LiDAR real
   │  │  │     │  │     └─ [IF isnan]: measurement[j] = sensor_range_min
   │  │  │     │  │
   │  │  │     │  └─ ┌─ PASO C: Evaluación de probabilidad
   │  │  │     │     │
   │  │  │     │     ├─ probability = 1.0
   │  │  │     │     │
   │  │  │     │     └─ ┌─ FOR j IN range(8):  ← BUCLE 3d: GAUSSIAN PRODUCT
   │  │  │     │        │
   │  │  │     │        ├─ gaussian_value = _gaussian(
   │  │  │     │        │      mu = measurement[j],
   │  │  │     │        │      sigma = sigma_z,
   │  │  │     │        │      x = z_hat[j]
   │  │  │     │        │  )
   │  │  │     │        │  
   │  │  │     │        │  ┌─ GAUSSIAN FORMULA:
   │  │  │     │        │  │  gaussian(μ, σ, x) = exp(-0.5*((x-μ)/σ)²)/(σ*√(2π))
   │  │  │     │        │  └─
   │  │  │     │        │
   │  │  │     │        └─ probability *= gaussian_value  ← Producto acumulativo
   │  │  │     │
   │  │  │     └─ RETURN probability (valor entre 0 y 1)
   │  │  │
   │  │  ├─ [END BUCLE 2]
   │  │  │
   │  │  ├─ ┌─ NORMALIZACIÓN DE PESOS
   │  │  │  │
   │  │  │  ├─ total_prob = sum(probabilities)
   │  │  │  ├─ probabilities /= total_prob
   │  │  │  └─ RESULTADO: Sum(probabilities) = 1.0
   │  │  │
   │  │  ├─ ┌─ REMUESTREO (RESAMPLING)
   │  │  │  │
   │  │  │  ├─ ┌─ Stratified Sampling:
   │  │  │  │  │
   │  │  │  │  ├─ u = uniform(0, 1/N)  ← Valor inicial aleatorio
   │  │  │  │  ├─ u_j = u + j/N for j in range(N)
   │  │  │  │  │
   │  │  │  │  └─ RESULTADO: N muestras uniformes distribuidas
   │  │  │  │
   │  │  │  ├─ ┌─ Cumulative Sum:
   │  │  │  │  │
   │  │  │  │  ├─ weight_circle = cumsum(probabilities)
   │  │  │  │  │  
   │  │  │  │  │  Ejemplo (N=4):
   │  │  │  │  │  probabilities = [0.1, 0.3, 0.4, 0.2]
   │  │  │  │  │  weight_circle = [0.1, 0.4, 0.8, 1.0]
   │  │  │  │  │
   │  │  │  │  └─ RESULTADO: CDF para búsqueda
   │  │  │  │
   │  │  │  ├─ ┌─ Digitize:
   │  │  │  │  │
   │  │  │  │  ├─ prominent_weights = np.digitize(u_j, weight_circle)
   │  │  │  │  │
   │  │  │  │  │  Busca dónde caen u_j en weight_circle
   │  │  │  │  │
   │  │  │  │  └─ RESULTADO: Índices de partículas a mantener
   │  │  │  │
   │  │  │  ├─ ┌─ Clipping:
   │  │  │  │  │
   │  │  │  │  ├─ prominent_weights = clip(prominent_weights, 0, N-1)
   │  │  │  │  │  ← Asegurar dentro de rango [0, N-1]
   │  │  │  │  │
   │  │  │  │  └─
   │  │  │  │
   │  │  │  └─ ┌─ Sample with Replacement:
   │  │  │     │
   │  │  │     ├─ self._particles = self._particles[prominent_weights]
   │  │  │     │
   │  │  │     │  EFECTO: Partículas con probabilidad alta se replican,
   │  │  │     │          partículas con probabilidad baja desaparecen
   │  │  │     │
   │  │  │     └─ RESULTADO: 1000 nuevas partículas
   │  │  │
   │  │  ├─ sense_time = perf_counter() - start_time
   │  │  │
   │  │  ├─ [IF enable_plot]: show("Sense", save_figure=True)
   │  │  │
   │  │  ├─ ┌─ PASO 3: ESTIMACIÓN DE POSE
   │  │  │  │
   │  │  │  ├─ start_time = perf_counter()
   │  │  │  │
   │  │  │  ├─ localized, pose = ParticleFilter.compute_pose()
   │  │  │  │  │
   │  │  │  │  ├─ ┌─ Project particles para clustering:
   │  │  │  │  │  │
   │  │  │  │  │  ├─ particles_5d = np.zeros((N, 5))
   │  │  │  │  │  │
   │  │  │  │  │  └─ ┌─ FOR i IN range(N):  ← BUCLE 4: PROYECTAR PARTÍCULAS
   │  │  │  │  │     │
   │  │  │  │  │     ├─ particles_5d[i, 0] = particle[i, 0]     ← x
   │  │  │  │  │     ├─ particles_5d[i, 1] = particle[i, 1]     ← y
   │  │  │  │  │     ├─ particles_5d[i, 2] = cos(particle[i, 2]) ← cos(θ)
   │  │  │  │  │     ├─ particles_5d[i, 3] = sin(particle[i, 2]) ← sin(θ)
   │  │  │  │  │     └─ particles_5d[i, 4] = particle[i, 2]     ← θ para media
   │  │  │  │  │
   │  │  │  │  ├─ ┌─ DBSCAN Clustering:
   │  │  │  │  │  │
   │  │  │  │  │  ├─ clustering = DBSCAN(eps=0.2, min_samples=5)
   │  │  │  │  │  ├─ clustering.fit(particles_5d[:, :4])  ← Usa solo (x,y,cos,sin)
   │  │  │  │  │  │
   │  │  │  │  │  ├─ labels = clustering.labels_  ← -1 para ruido, 0,1,2... para clusters
   │  │  │  │  │  ├─ indexes = clustering.core_sample_indices_  ← Índices de puntos core
   │  │  │  │  │  │
   │  │  │  │  │  ├─ n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
   │  │  │  │  │  │
   │  │  │  │  │  └─
   │  │  │  │  │
   │  │  │  │  ├─ ┌─ ANÁLISIS DE CLUSTERS:
   │  │  │  │  │  │
   │  │  │  │  │  ├─ [IF n_clusters == 1]:  ← ¡LOCALIZED!
   │  │  │  │  │  │  │
   │  │  │  │  │  │  ├─ localized = True
   │  │  │  │  │  │  │
   │  │  │  │  │  │  ├─ cluster_particles = self._particles[indexes]
   │  │  │  │  │  │  │
   │  │  │  │  │  │  ├─ ┌─ Compute cluster mean:
   │  │  │  │  │  │  │  │
   │  │  │  │  │  │  │  ├─ x_mean = mean(cluster_particles[:, 0])
   │  │  │  │  │  │  │  ├─ y_mean = mean(cluster_particles[:, 1])
   │  │  │  │  │  │  │  │
   │  │  │  │  │  │  │  ├─ ┌─ Circular mean para θ:
   │  │  │  │  │  │  │  │  │
   │  │  │  │  │  │  │  │  ├─ sin_mean = mean(sin(cluster_particles[:, 2]))
   │  │  │  │  │  │  │  │  ├─ cos_mean = mean(cos(cluster_particles[:, 2]))
   │  │  │  │  │  │  │  │  │
   │  │  │  │  │  │  │  │  ├─ theta_mean = atan2(sin_mean, cos_mean) % 2π
   │  │  │  │  │  │  │  │  │
   │  │  │  │  │  │  │  │  └─
   │  │  │  │  │  │  │  │
   │  │  │  │  │  │  │  └─ pose = (x_mean, y_mean, theta_mean)
   │  │  │  │  │  │  │
   │  │  │  │  │  │  ├─ ┌─ Reduce particles para tracking:
   │  │  │  │  │  │  │  │
   │  │  │  │  │  │  │  └─ self._particle_count = 25  ← Menos partículas en fase tracking
   │  │  │  │  │  │  │
   │  │  │  │  │  │  └─
   │  │  │  │  │  │
   │  │  │  │  │  ├─ [ELIF n_clusters > 1]:  ← Still exploring
   │  │  │  │  │  │  │
   │  │  │  │  │  │  ├─ localized = False
   │  │  │  │  │  │  │
   │  │  │  │  │  │  └─ ┌─ Increase particles:
   │  │  │  │  │  │     │
   │  │  │  │  │  │     └─ self._particle_count = max(100 * n_clusters, 100)
   │  │  │  │  │  │
   │  │  │  │  │  ├─ [ELSE]:  ← No clear cluster (all noise)
   │  │  │  │  │  │  │
   │  │  │  │  │  │  ├─ localized = False
   │  │  │  │  │  │  │
   │  │  │  │  │  │  └─ ┌─ Keep particles:
   │  │  │  │  │  │     │
   │  │  │  │  │  │     └─ self._particle_count unchanged
   │  │  │  │  │  │
   │  │  │  │  │  └─
   │  │  │  │  │
   │  │  │  │  └─ RETURN (localized, pose)
   │  │  │  │
   │  │  │  ├─ clustering_time = perf_counter() - start_time
   │  │  │  │
   │  │  │  └─ logger.info("Clustering time: {:.3f}s".format(clustering_time))
   │  │  │
   │  │  └─
   │  │
   │  ├─ self._steps += 1
   │  │
   │  └─ RETURN pose
   │
   ├─ _publish_pose_estimate(x_h, y_h, theta_h)
   │  │
   │  ├─ msg = PoseStamped()
   │  │
   │  ├─ msg.header.stamp = self.get_clock().now()
   │  │
   │  ├─ msg.header.frame_id = "map"
   │  │
   │  ├─ ┌─ [IF self._localized]:
   │  │  │
   │  │  ├─ msg.pose.position.x = x_h
   │  │  ├─ msg.pose.position.y = y_h
   │  │  │
   │  │  ├─ ┌─ Convert Euler to Quaternion:
   │  │  │  │
   │  │  │  ├─ (qx, qy, qz, qw) = euler2quat(roll=0, pitch=0, yaw=theta_h)
   │  │  │  │
   │  │  │  └─
   │  │  │
   │  │  ├─ msg.pose.orientation.x = qx
   │  │  ├─ msg.pose.orientation.y = qy
   │  │  ├─ msg.pose.orientation.z = qz
   │  │  ├─ msg.pose.orientation.w = qw
   │  │  │
   │  │  └─
   │  │
   │  ├─ msg.localized = self._localized
   │  │
   │  └─ pose_publisher.publish(msg)
   │
   └─ [END Callback]

═══════════════════════════════════════════════════════════════════════════════
```

---

## 4. ESTRUCTURA DE BUCLES ANIDADOS

### **Resumen de Bucles**

```
Llamada a _compute_pose_callback()
│
├─ _execute_motion_step(v, w)
│  │
│  └─ ParticleFilter.move(v, w)
│     └─ BUCLE 1: for particle in self._particles (1000 iteraciones)
│        ├─ Actualizar pose de partícula
│        └─ Verificar colisión con mapa
│
└─ _execute_measurement_step(z_scan)  [cada 10 ciclos]
   │
   └─ ParticleFilter.resample(z_scan)
      │
      ├─ BUCLE 2: for particle in self._particles (1000 iteraciones)
      │  │
      │  └─ _measurement_probability(z_scan, particle)
      │     │
      │     ├─ _sense(particle_pose)
      │     │  │
      │     │  └─ _lidar_rays(pose, ray_indices)
      │     │     │
      │     │     └─ BUCLE 3: for ray_index in [0,30,60,90,120,150,180,210] (8 rayos)
      │     │        ├─ Proyectar rayo desde robot hacia sensor_range_max
      │     │        └─ Devolver 8 segmentos
      │     │
      │     ├─ [Para cada segmento de rayo]
      │     │  └─ map.check_collision() ← Calcula distancia hasta obstáculo
      │     │
      │     └─ BUCLE 3b: for (measurement, predicted) in zip(z_scan_samples, z_hat) (8 pares)
      │        └─ probability *= _gaussian(measurement, sigma_z, predicted)
      │
      ├─ Normalizar probabilidades
      │
      └─ Remuestreo estratificado
         ├─ weight_circle = cumsum(probabilities)
         ├─ u_j = random samples
         ├─ indices = digitize(u_j, weight_circle)
         └─ self._particles = self._particles[indices]  ← Replicate/eliminate
      
      └─ compute_pose()
         │
         └─ BUCLE 4: for particle in self._particles (1000 iteraciones)
            └─ Proyectar a 5D: (x, y, cos(θ), sin(θ), θ)
         
         └─ DBSCAN.fit(particles_5d[:, :4])
            └─ Cluster based on spatial proximity
         
         └─ [If n_clusters == 1]: LOCALIZED!
            ├─ Compute cluster mean
            └─ Reduce particles to 25
```

---

## 5. ORDEN DE EJECUCIÓN DETALLADO - POR ITERACIÓN

### **ITERACIÓN 1: Motion Update (SIEMPRE)**

```
Tiempo: t=0
Evento: /odometry + /scan llegan

_compute_pose_callback() inicia
  ↓
  Extract v, w, scan
  ↓
  _execute_motion_step(v, w)
    ↓
    move(v, w)  ← SIEMPRE
      ↓
      FOR i=0 TO 999:  ← 1000 actualizaciones
        - Generar ruido (σ_v, σ_w)
        - Actualizar (x, y, θ)
        - Verificar colisión
      ↓
    _particles actualizado
    ↓
    Visualizar (opcional)
    ↓
  FIN _execute_motion_step
```

### **ITERACIÓN 2: Measurement Update (CADA 10 ITERACIONES)**

```
Tiempo: t=10
Evento: /odometry + /scan llegan (la 10ª vez)

_compute_pose_callback() inicia
  ↓
  Extract v, w, scan
  ↓
  _execute_motion_step(v, w)
    ↓
    move(v, w)  ← SIEMPRE
    ↓
  _execute_measurement_step(scan)  ← AHORA SÍ
    ↓
    resample(scan)  ← MEASUREMENT UPDATE
      ↓
      FOR i=0 TO 999:  ← 1000 evaluaciones de probabilidad
        ↓
        _measurement_probability(scan, particle_i)
          ↓
          _sense(particle_pose)  
            ↓
            _lidar_rays(pose, [0,30,60,90,120,150,180,210])
              ↓
              FOR j=0 TO 7:  ← 8 rayos
                - Proyectar rayo
              ↓
              RETURN 8 rayos
            ↓
          FOR j=0 TO 7:  ← 8 distancias
            - map.check_collision(rayo_j)
            - z_hat[j] = distancia
          ↓
          RETURN z_hat (8 valores)
        ↓
        FOR j=0 TO 7:  ← 8 medidas reales
          - z_scan_subsampled[j] = z_scan[30*j]
        ↓
        probability = 1.0
        FOR j=0 TO 7:  ← 8 gaussianas
          - probability *= gaussian(z_scan[j], sigma_z, z_hat[j])
        ↓
        probabilities[i] = probability
      ↓
      Normalize & Resample
        ↓
        weight_circle = cumsum(probabilities)
        ↓
        FOR i=0 TO 999:
          - u_i = random sample
          - idx = digitize(u_i, weight_circle)
          - new_particles[i] = particles[idx]
        ↓
      ↓
    compute_pose()
      ↓
      FOR i=0 TO 999:
        - Project to 5D
      ↓
      DBSCAN.fit()
      ↓
      IF n_clusters == 1:
        - localized = True
        - pose = cluster_mean
        - self._particle_count = 25  ← IMPORTANTE!
      ↓
      RETURN pose
    ↓
  publish_pose()
```

---

## 6. COMPLEJIDAD COMPUTACIONAL

```
┌──────────────────────────────────────────────────────────┐
│           ANÁLISIS DE COMPLEJIDAD POR CICLO              │
└──────────────────────────────────────────────────────────┘

move() (SIEMPRE):
  ├─ FOR 1000 partículas:
  │  └─ Actualización cinemática: O(1)
  │  └─ Collision check: O(1)
  ├─ Complejidad: O(N) = O(1000)
  └─ Tiempo: ~10-20 ms


resample() (CADA 10 CICLOS):
  ├─ FOR 1000 partículas:
  │  └─ _measurement_probability():
  │     ├─ _sense(): O(1)
  │     ├─ _lidar_rays(): O(8)
  │     │  └─ FOR 8 rayos:
  │     │     └─ Proyección: O(1)
  │     ├─ FOR 8 rayos:
  │     │  └─ map.check_collision(): O(n_segments)
  │     └─ FOR 8 medidas:
  │        └─ _gaussian(): O(1)
  ├─ Complejidad: O(N × 8 × collision_checks) = O(8000)
  └─ Tiempo: ~300-500 ms


compute_pose():
  ├─ FOR 1000 partículas:
  │  └─ Proyectar a 5D: O(1)
  ├─ DBSCAN.fit(): O(N²) worst case, O(N log N) average
  ├─ Clustering analysis: O(N)
  ├─ Complejidad: O(N log N) to O(N²)
  └─ Tiempo: ~50-200 ms


TOTAL POR CICLO:
  - Sin measurement: ~20 ms
  - Con measurement (cada 10): ~350-700 ms
  - Promedio con factor 1/10: ~75 ms


Con 1000 partículas:
  - move(): 1000 updates + 1000 collision checks
  - resample() (1/10): 8000 distance checks + 8000 gaussians
  - compute_pose(): DBSCAN en 1000 puntos
```

---

## 7. FLUJO DE DATOS A TRAVÉS DE ESTRUCTURAS

```
ParticleFilterNode
    ↓
    self.pf_instance ← ParticleFilter
    ↓
    ParticleFilter._particles
        ↓
        NumPy array (1000, 3)
        ├─ [:, 0] = x coordinates
        ├─ [:, 1] = y coordinates
        └─ [:, 2] = theta angles
    ↓
    Move step:
        self._particles[i, 0] += update_x
        self._particles[i, 1] += update_y
        self._particles[i, 2] += update_theta
    ↓
    Resample step:
        probabilities = compute for each particle
        self._particles = self._particles[prominent_weights]
    ↓
    State estimation:
        x_est = mean(self._particles[:, 0])
        y_est = mean(self._particles[:, 1])
        theta_est = atan2(..., ...)
    ↓
    PoseStamped message
        pose.position.x = x_est
        pose.position.y = y_est
        pose.orientation = quaternion(theta_est)
```

---

## 8. COMPARATIVA: sim_ws vs tb3_ws

```
╔═══════════════════════════════════════════════════════════════╗
║                     SIM_WS vs TB3_WS                          ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║ SINCRONIZACIÓN:                                               ║
║ ├─ sim_ws:  ApproximateTimeSynchronizer                      ║
║ │           └─ Callback único: (odom, scan) sincronizados    ║
║ │                                                             ║
║ └─ tb3_ws:  Dos callbacks + Timer                            ║
║             ├─ _callback_odometry_saving_history()           ║
║             ├─ _callback_scan_saving_history()               ║
║             └─ _timer_callback() cada 2 segundos             ║
║                                                               ║
║ PROCESAMIENTO:                                                ║
║ ├─ sim_ws:  move() + resample() en el mismo callback         ║
║ │                                                             ║
║ └─ tb3_ws:  Acumula múltiples move() antes de resample()    ║
║             ├─ FOR odometry in _odometry_estimate_list:      ║
║             │  └─ move(v, w)  [múltiples]                    ║
║             └─ resample(scan)  [una sola]                    ║
║                                                               ║
║ VENTAJA TB3:                                                  ║
║ └─ Procesa múltiples updates de odometría (30Hz) antes de    ║
║    una actualización de medida de scan (10Hz)                ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
```

---

## 9. EJEMPLO CONCRETO: SIGUIENTE PARTÍCULA

```
Partícula i = 500:

Estado inicial: (2.5, 3.1, 0.785)  [x=2.5m, y=3.1m, θ=π/4]

Velocidades medidas:
  v = 0.3 m/s
  w = 0.1 rad/s

PASO 1: move()
  ├─ noise_v = gaussiano(0, 0.05) = +0.01
  ├─ noise_w = gaussiano(0, 0.15) = -0.02
  │
  ├─ x_vel = cos(0.785) * (0.3 + 0.01) = 0.707 * 0.31 = 0.219
  ├─ y_vel = sin(0.785) * (0.3 + 0.01) = 0.707 * 0.31 = 0.219
  │
  ├─ Δx = 0.219 * 0.05 = 0.01095
  ├─ Δy = 0.219 * 0.05 = 0.01095
  ├─ Δθ = (0.1 - 0.02) * 0.05 = 0.004
  │
  ├─ particle[0] = 2.5 + 0.01095 = 2.51095
  ├─ particle[1] = 3.1 + 0.01095 = 3.11095
  ├─ particle[2] = (0.785 + 0.004) % 2π = 0.789
  │
  ├─ Verificar colisión: No hay
  │
  ├─ Nuevo estado: (2.51095, 3.11095, 0.789)
  └─

PASO 2: resample() [cada 10 ciclos]
  ├─ z_hat = _sense((2.51095, 3.11095, 0.789))
  │  │
  │  └─ FOR rayo j IN [0,1,2,3,4,5,6,7]:
  │     ├─ ray_angle = 1.5 * j degrees
  │     ├─ Proyectar desde (2.51, 3.11) en ángulo ray_angle
  │     └─ Chequear colisión hasta sensor_range_max (3.5m)
  │
  │  Resultado (ejemplo): z_hat = [1.2, 0.8, 1.5, 2.1, 0.9, 1.3, 1.1, 0.7]
  │
  ├─ z_scan_subsampled = [z_scan[0], z_scan[30], z_scan[60], ...]
  │  Resultado: [1.25, 0.75, 1.48, 2.15, 0.95, 1.28, 1.05, 0.68]
  │
  ├─ probability = 1.0
  │  FOR j IN [0,1,2,3,4,5,6,7]:
  │     gaussian_j = gaussian(μ=z_scan[j], σ=0.1, x=z_hat[j])
  │     probability *= gaussian_j
  │
  │  gaussian_0 = exp(-0.5*((1.25-1.2)/0.1)²) / (0.1*√(2π)) ≈ 3.95
  │  gaussian_1 = ...
  │  ...
  │  probability ≈ 3.95 * ... * ... ≈ 2.14e-8
  │
  ├─ RETURN probability
  └─

PASO 3: compute_pose()
  ├─ [IF n_clusters == 1]:
  │  ├─ x_mean = 2.51 (aprox)
  │  ├─ y_mean = 3.11 (aprox)
  │  └─ theta_mean = 0.789 (aprox)
  │
  ├─ RETURN (True, (2.51, 3.11, 0.789))
  └─

RESULTADO FINAL:
  Partícula original:  (2.5,   3.1,   0.785)
  Partícula actual:    (2.511, 3.111, 0.789)
  Pose estimada:       (2.51,  3.11,  0.789)  [media de cluster]
```

---

## 10. TABLA RESUMEN DE FUNCIONES

| Función | Llamada desde | Bucles | Frecuencia | Tiempo |
|---------|---------------|--------|-----------|--------|
| `__init__` | ROS 2 Lifecycle | 1 (partículas init) | Una vez | ~100ms |
| `move()` | `_execute_motion_step()` | N=1000 | Cada callback | ~15ms |
| `resample()` | `_execute_measurement_step()` | N×8=8000 | Cada 10 callbacks | ~400ms |
| `compute_pose()` | `_execute_measurement_step()` | N=1000 (DBSCAN) | Cada 10 callbacks | ~100ms |
| `_measurement_probability()` | `resample()` | 1 | 8000 veces por resample | ~0.05ms |
| `_sense()` | `_measurement_probability()` | 8 (rayos) | 1000×10 = 10000 veces | ~0.01ms |
| `_lidar_rays()` | `_sense()` | 8 | 10000 veces | ~0.005ms |
| `_gaussian()` | `_measurement_probability()` | 8 | 80000 veces | ~0.0001ms |

═══════════════════════════════════════════════════════════════════════════════
