# Arquitectura ROS 2 - Robots Móviles Autónomos

## Índice
1. [Práctica 1: Simulación CoppeliaSim](#práctica-1-simulación-coppelia-sim)
2. [Práctica 2: Localización con Filtro de Partículas](#práctica-2-localización-con-filtro-de-partículas)
3. [Práctica 3: Planificación y Control](#práctica-3-planificación-y-control)
4. [Práctica 4: Robot Real TurtleBot3](#práctica-4-robot-real-turtlebot3)
5. [Resumen Comparativo](#resumen-comparativo)

---

## Práctica 1: Simulación CoppeliaSim

### Objetivo
Integración del simulador CoppeliaSim con ROS 2. Adquisición de datos de odometría y LiDAR. Comunicación básica entre simulador y ROS.

### Nodos Involucrados

#### CoppeliaSimNode
- **Tipo**: LifecycleNode
- **Ubicación**: `sim_ws/src/amr_simulation/amr_simulation/coppeliasim_node.py`
- **Función**: Interfaz principal con el simulador CoppeliaSim

**Tópicos Publicados:**
| Topic | Tipo | Descripción | Frecuencia |
|-------|------|-------------|-----------|
| `/odometry` | `nav_msgs/Odometry` | Pose + velocidad del robot | 50 Hz |
| `/scan` | `sensor_msgs/LaserScan` | Medidas LiDAR 360° | 50 Hz |

**Tópicos Suscritos:**
| Topic | Tipo | Descripción | Sincronización |
|-------|------|-------------|-----------------|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Comando de velocidad | No |
| `/pose` | `amr_msgs/PoseStamped` | Pose estimada (opcional) | No |

### Diagrama de Flujo - P1
```
┌─────────────────────────┐
│   CoppeliaSim Node      │
│   (LifecycleNode)       │
└────┬────────────┬───────┘
     │            │
     ▼            ▼
 /odometry    /scan
     │            │
     └─────┬──────┘
           │
     ┌─────▼──────┐
     │ Aplicación │
     │ (Lectura)  │
     └────────────┘

     /cmd_vel ◀─── (entrada opcional)
     /pose    ◀─── (entrada opcional)
```

### Características Clave
- Comunicación socket con CoppeliaSim
- Mundo simulado: "project"
- Posición inicial: (-1, -1, 90°)
- Obstáculos definidos en escena
- Frecuencia: 50 Hz

---

## Práctica 2: Localización con Filtro de Partículas

### Objetivo
Implementar sistema de localización que combine odometría y LiDAR. Estimar la pose del robot en el mapa con confianza de localización.

### Nodos Involucrados

#### CoppeliaSimNode (heredado de P1)
- Continúa proporcionando `/odometry` y `/scan`

#### ParticleFilterNode ⭐
- **Tipo**: LifecycleNode
- **Ubicación**: `sim_ws/src/amr_localization/amr_localization/particle_filter_node.py`
- **Función**: Localización bayesiana con filtro de partículas

**Tópicos Publicados:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/pose` | `amr_msgs/PoseStamped` | Pose estimada + flag de localización |

**Tópicos Suscritos (Sincronizados):**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/odometry` | `nav_msgs/Odometry` | Sincronizado con /scan |
| `/scan` | `sensor_msgs/LaserScan` | Sincronizado con /odometry |

**Sincronización**: MessageFilters con ApproximateTimeSynchronizer (slop: 0.1s)

### Parámetros del Filtro
```
particles: 1000              # Cantidad de partículas
sigma_v: 0.05               # Ruido velocidad lineal (m/s)
sigma_w: 0.15               # Ruido velocidad angular (rad/s)
sigma_z: 0.10               # Ruido medida LiDAR (m)
update_frequency: 10 Hz     # Actualización del filtro
```

### Algoritmo del Filtro de Partículas

```
┌──────────────────────────────────────┐
│    Ciclo del Filtro de Partículas    │
└──────────────────────────────────────┘

1. PREDICCIÓN (Motion Model)
   - Entrada: /odometry
   - Actualiza partículas según movimiento
   - p(x_k | u_k)
   
2. ACTUALIZACIÓN (Measurement Model)
   - Entrada: /scan
   - Calcula peso de cada partícula
   - p(x_k | z_k)
   
3. REMUESTREO (Resampling)
   - Descarta partículas con bajo peso
   - Duplica partículas con alto peso
   - Evita "degeneración"
   
4. ESTIMACIÓN (Pose Estimation)
   - Calcula media ponderada
   - Publica /pose
   - Flag: localized si varianza < threshold
```

### Diagrama de Flujo - P2
```
  CoppeliaSimNode
  ┌──────┬──────┐
  │      │      │
  ▼      ▼      ▼
/odom  /scan  /cmd_vel
  │      │
  └──┬───┘
     │ (sincronizados)
     ▼
[MessageFilter]
     │
     ▼
┌──────────────────────────┐
│  ParticleFilterNode      │
│  - Predicción (odometría)│
│  - Actualización (LiDAR) │
│  - Remuestreo           │
│  - Estimación            │
└──────────────────────────┘
     │
     ▼
  /pose (amr_msgs/PoseStamped)
  ├─ pose (x, y, θ)
  └─ localized (bool)
```

### Características Clave
- Sincronización temporal de múltiples sensores
- Modelo de movimiento cinemático
- Actualización basada en matching de scans
- Detección automática de localización
- Número de partículas: 1000 (simulación)

---

## Práctica 3: Planificación y Control

### Objetivo
Sistema completo de navegación autónoma. Planificación de caminos con PRM. Control mediante Pure Pursuit o Wall Following.

### Nodos Involucrados

#### CoppeliaSimNode (heredado)
- Proporciona sensores

#### ParticleFilterNode (heredado de P2)
- Proporciona `/pose` estimada

#### PRMNode ⭐
- **Tipo**: LifecycleNode
- **Ubicación**: `sim_ws/src/amr_planning/amr_planning/prm_node.py`
- **Función**: Planificación de caminos con Roadmaps Probabilísticos

**Tópicos Publicados:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/path` | `nav_msgs/Path` | Secuencia de waypoints planificados |

**Tópicos Suscritos:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/pose` | `amr_msgs/PoseStamped` | Pose actual (trigger de planificación) |

**Parámetros:**
```
n_samples: 250              # Nodos del roadmap
connection_dist: 1.5 m      # Radio de conexión entre nodos
grid_resolution: 0.05 m     # Resolución de verificación de colisiones
goal_x: -0.6 m              # Meta X (fija)
goal_y: 1.0 m               # Meta Y (fija)
```

**Algoritmo PRM:**
```
1. MUESTREO (Sampling)
   - Generar 250 puntos aleatorios
   - Descartar si están en colisión
   
2. CONECTIVIDAD (Connection)
   - Conectar cada nodo con vecinos cercanos (< 1.5 m)
   - Verificar colisiones en segmentos
   
3. BÚSQUEDA (Path Search)
   - Buscar camino desde pose actual a meta
   - Algoritmo: Dijkstra o A*
   - Retornar secuencia de waypoints
```

#### PurePursuitNode ⭐
- **Tipo**: LifecycleNode
- **Ubicación**: `sim_ws/src/amr_control/amr_control/pure_pursuit_node.py`
- **Función**: Seguimiento de camino basado en Pure Pursuit

**Tópicos Publicados:**
| Topic | Tipo | Descripción | Frecuencia |
|-------|------|-------------|-----------|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Comando de velocidad | 50 Hz |

**Tópicos Suscritos:**
| Topic | Tipo | Descripción | Sincronización |
|-------|------|-------------|-----------------|
| `/pose` | `amr_msgs/PoseStamped` | Pose actual | No |
| `/path` | `nav_msgs/Path` | Camino a seguir | No |

**Parámetros:**
```
lookahead_distance: 0.5 m   # Distancia de look-ahead
v_max: 0.5 m/s              # Velocidad máxima
control_frequency: 50 Hz    # Frecuencia de control
kp_angular: 1.0             # Ganancia de control angular
```

**Algoritmo Pure Pursuit:**
```
1. Encontrar punto objetivo en el camino a distancia lookahead
2. Calcular ángulo hacia ese punto
3. Calcular curvatura requerida: kappa = 2 * sin(theta) / lookahead
4. Generar comando: v_linear = v_max, w_angular = v_max * kappa
5. Publicar comando en /cmd_vel
```

#### WallFollowerNode ⭐
- **Tipo**: LifecycleNode
- **Ubicación**: `sim_ws/src/amr_control/amr_control/wall_follower_node.py`
- **Función**: Control reactivo siguiendo paredes

**Tópicos Publicados:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Comando de velocidad |

**Tópicos Suscritos (Sincronizados):**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/odometry` | `nav_msgs/Odometry` | Sincronizado |
| `/scan` | `sensor_msgs/LaserScan` | Sincronizado |
| `/pose` | `amr_msgs/PoseStamped` | Opcional si localización activa |

**Parámetros:**
```
target_distance: 0.30 m     # Distancia objetivo a la pared
v_linear: 0.3 m/s           # Velocidad líneal constante
kp: 0.8                      # Ganancia proporcional
scan_range: [0.2, 3.5] m    # Rango válido del LiDAR
```

**Algoritmo Wall Following:**
```
1. Buscar obstáculos en rango lateral del LiDAR
2. Medir distancia perpendicular a la pared
3. Error = target_distance - measured_distance
4. Corrección angular: w = kp * error
5. Enviar comando: v_linear = 0.3 m/s, w_angular = corrección
```

### Diagrama de Flujo - P3 (Completo)
```
                CoppeliaSimNode
                ┌──────┬──────┐
                │      │      │
                ▼      ▼      ▼
            /odom  /scan  /cmd_vel
                │      │       ▲
                └──┬───┘       │
                   │           │
         [MessageFilter]       │
                   │           │
                   ▼           │
            ParticleFilter     │
             (P2 module)       │
                   │           │
                   ▼           │
              /pose            │
              ┌─┴────────┐     │
              │          │     │
              ▼          ▼     │
            PRMNode   ┌────────┴────┐
             │        │             │
             ▼        │             │
           /path      │             │
             │        ▼             ▼
             │    PurePursuitNode WallFollowerNode
             │        │             │
             └────┬───┘             │
                  │                 │
                  └────────┬────────┘
                           │
                           ▼
                      /cmd_vel ──→ CoppeliaSim
```

### Modos de Control - P3
```
┌─────────────────────────────────┐
│   DECISION: ¿Tiene camino?      │
└────┬────────────────────────┬───┘
     │ SÍ (path disponible)   │ NO (sin path)
     ▼                        ▼
PurePursuitNode         WallFollowerNode
(Seguimiento             (Control reactivo)
 planificado)

- Sigue waypoints exactos    - Sigue pared
- Óptimo si camino válido    - Exploración
- Requiere /path             - Solo necesita /scan
```

---

## Práctica 4: Robot Real TurtleBot3

### Objetivo
Implementar el mismo sistema de localización y control en robot físico. Integración con hardware real, teleopeación manual y monitoreo de objetivos.

### Cambios Principales vs. Simulación

| Aspecto | Simulación (P1-P3) | Robot Real (P4) |
|---------|-------------------|-----------------|
| **Fuente Odometría** | CoppeliaSim | TurtleBot3 `/odom` |
| **Fuente LiDAR** | Simulador | LDS-01 (sensor real) |
| **Tipo cmd_vel** | `geometry_msgs/TwistStamped` | `geometry_msgs/Twist` |
| **Nodos extras** | — | Teleopeación, Monitor |
| **Partículas PF** | 1000 | 2000 |
| **Ruido modelo** | Bajo | Alto (parámetros > 2x) |
| **Driver middleware** | Socket TCP | ROS 2 nativo |

### Nodos Específicos de P4

#### OdometryNode (Nuevo en P4)
- **Tipo**: Regular Node
- **Ubicación**: `tb3_ws/src/amr_turtlebot3/amr_turtlebot3/odometry_node.py`
- **Función**: Adaptador de odometría

**Tópicos Publicados:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/odometry` | `nav_msgs/Odometry` | Odometría enriquecida con velocidades |

**Tópicos Suscritos:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Odometría nativa del TurtleBot3 |

**Función**: Calcula velocidades derivando posición, mantiene compatibilidad con P1-P3

#### ParticleFilterNode (Versión P4)
**Cambios vs. P2-P3:**
```
Parámetros aumentados para robot real:
- particles: 2000 (vs. 1000 en sim)
- sigma_v: 0.1 (vs. 0.05) - más ruido en velocidad real
- sigma_w: 0.25 (vs. 0.15) - más ruido angular
- sigma_z: 0.15 (vs. 0.10) - sensor real menos preciso
- update_frequency: 5 Hz (vs. 10 Hz) - procesamiento más lento
```

#### TeleoperationNode (Nuevo en P4) ⭐
- **Tipo**: Regular Node
- **Ubicación**: `tb3_ws/src/amr_teleoperation/amr_teleoperation/teleoperation_node.py`
- **Función**: Control manual del robot

**Tópicos Publicados:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocidad desde teclado |

**Tópicos Suscritos:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/listen_key` | `amr_msgs/KeyPressed` | Teclas presionadas |
| `/scan` | `sensor_msgs/LaserScan` | Detección de obstáculos |

**Mapeo de Teclas:**
```
w        → Adelante (v_linear > 0)
s        → Atrás (v_linear < 0)
a        → Giro izquierda (w_angular > 0)
d        → Giro derecha (w_angular < 0)
ESPACIO  → Parar completamente
```

**Sistema de Seguridad:**
```
Detección de obstáculos:
- Si distancia_obstáculo < 0.25 m → PARAR automáticamente
- Escanea rango frontal del LiDAR
- Previene colisiones en teleopeación manual
```

#### KeyboardNode (Nuevo en P4)
- **Tipo**: Regular Node
- **Ubicación**: `tb3_ws/src/amr_teleoperation/amr_teleoperation/keyboard_node.py`
- **Función**: Captura eventos del teclado

**Tópicos Publicados:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/listen_key` | `amr_msgs/KeyPressed` | Eventos de tecla |

**Dependencias**: `pynput` (captura de teclado portable)

#### MonitoringNode (Nuevo en P4) ⭐
- **Tipo**: LifecycleNode
- **Ubicación**: `tb3_ws/src/amr_turtlebot3/amr_turtlebot3/monitoring_node.py`
- **Función**: Monitoreo de logro de objetivos

**Tópicos Suscritos:**
| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/pose` | `amr_msgs/PoseStamped` | Pose estimada |

**Verificaciones:**
```
- Distancia a meta < threshold (ej: 0.15 m)
- Tiempo en meta > confirmación (ej: 2 segundos)
- Registra tiempo de llegada
- Valida localización (localized flag)
```

### Diagrama de Flujo - P4 (Completo)
```
              TurtleBot3 Hardware
              ┌──────┬──────┬─────┐
              │      │      │     │
              ▼      ▼      ▼     ▼
            /odom  /scan  /cmd_vel Keyboard
              │      │       ▲      │
              │      │       │      │
        OdometryNode│       │    KeyboardNode
              │      │       │      │
              ▼      │       │      ▼
            /odometry│       │  /listen_key
              │      │       │      │
              └──┬───┘       │      ▼
                 │           │  TeleoperationNode
         [MessageFilter]     │      │
                 │           │      │
                 ▼           │      │
            ParticleFilter   │      │
                 │           │  Safety Check
                 ▼           │ (obstacle detection)
              /pose          │
              ┌─┴────────────┼──────┐
              │              │      │
              ▼              ▼      ▼
            PRMNode    PurePursuitNode TeleoperationNode
             │              │           │
             ▼              │           │
           /path            │           │
             │              │           │
             └──────┬───────┘           │
                    │                   │
                    └───────┬───────────┘
                            │
                            ▼
                       /cmd_vel (Twist)
                            │
                            ▼
                    TurtleBot3 Motors
                            │
                            ▼
                        Movimiento
                            │
                            ▼
                    MonitoringNode
                    (Verifica meta)
```

### Flujos Alternativos en P4

**FLUJO 1: Navegación Autónoma**
```
1. ParticleFilterNode estima pose
2. PRMNode planifica camino a meta
3. PurePursuitNode genera comandos de velocidad
4. Robot sigue camino
5. MonitoringNode detecta llegada
```

**FLUJO 2: Teleopeación Manual**
```
1. KeyboardNode captura entrada de usuario
2. TeleoperationNode mapea teclas a velocidades
3. Sistema de seguridad verifica obstáculos
4. Si obstáculo cercano → PARAR
5. Si OK → Enviar comando a motor
```

**FLUJO 3: Cambio Dinámico de Modo**
```
- Usuario puede cambiar entre autónomo y manual
- ParticleFilterNode siempre activo (localización)
- MonitoringNode siempre monitorea
- Controlador (PP o Tele) es intercambiable
```

---

## Resumen Comparativo

### Matriz de Tópicos por Práctica

| Tópico | P1 | P2 | P3 | P4 |
|--------|----|----|----|----|
| `/odometry` | **Pub** | Sub (sync) | Sub (sync) | Sub (enhanced) |
| `/scan` | **Pub** | Sub (sync) | Sub (sync) | Sub |
| `/cmd_vel` | Sub | — | Sub | Sub |
| `/pose` | — | **Pub** | **Pub** | **Pub** |
| `/path` | — | — | **Pub** | **Pub** |
| `/listen_key` | — | — | — | **Pub** |
| `/odom` | — | — | — | Sub (nativo TB3) |

### Matriz de Nodos por Práctica

| Nodo | Tipo | P1 | P2 | P3 | P4 |
|------|------|----|----|----|----|
| CoppeliaSimNode | LifecycleNode | ✓ | ✓ | ✓ | — |
| ParticleFilterNode | LifecycleNode | — | ✓ | ✓ | ✓ |
| PRMNode | LifecycleNode | — | — | ✓ | ✓ |
| PurePursuitNode | LifecycleNode | — | — | ✓ | ✓ |
| WallFollowerNode | LifecycleNode | — | — | ✓ | ✓ |
| OdometryNode | Regular Node | — | — | — | ✓ |
| TeleoperationNode | Regular Node | — | — | — | ✓ |
| KeyboardNode | Regular Node | — | — | — | ✓ |
| MonitoringNode | LifecycleNode | — | — | — | ✓ |
| LifecycleManager | Regular Node | ✓ | ✓ | ✓ | ✓ |

### Evolución de Complejidad

```
P1: 2 nodos (Sim + Manager)
    └─ Arquitectura simple: solo sensores

P2: 3 nodos (Sim + PF + Manager)
    └─ Agregado: localización

P3: 5 nodos (Sim + PF + PRM + Control(2) + Manager)
    └─ Agregado: planificación y múltiples controladores

P4: 9 nodos (HW + Odom + PF + PRM + Control(2) + Tele + Monitor + Manager)
    └─ Agregado: teleopeación, monitoreo, driver de HW
    └─ Cambios: mensajes (TwistStamped→Twist), parámetros aumentados
```

### Parámetros Clave Comparativa

```
LOCALIZATION (ParticleFilterNode)
├─ Partículas
│  ├─ P2-P3: 1000
│  └─ P4: 2000
├─ Ruido lineal (sigma_v)
│  ├─ P2-P3: 0.05 m/s
│  └─ P4: 0.1 m/s
├─ Ruido angular (sigma_w)
│  ├─ P2-P3: 0.15 rad/s
│  └─ P4: 0.25 rad/s
└─ Ruido medida (sigma_z)
   ├─ P2-P3: 0.10 m
   └─ P4: 0.15 m

PLANNING (PRMNode)
├─ Muestras: 250 (todas)
├─ Radio conexión: 1.5 m (todas)
└─ Resolución grid: 0.05 m (todas)

CONTROL
├─ Pure Pursuit
│  ├─ Lookahead: 0.5 m
│  ├─ v_max: 0.5 m/s
│  └─ Frecuencia: 50 Hz
└─ Wall Following
   ├─ Target dist: 0.30 m
   ├─ v_linear: 0.3 m/s
   └─ Kp: 0.8
```

---

## Tipos de Mensajes Personalizados

### amr_msgs/PoseStamped
```
Header header
Pose pose
  - Position
    - x: float64
    - y: float64
    - z: float64
  - Orientation (quaternion)
    - x, y, z, w: float64
bool localized  # Flag de confianza
```

### amr_msgs/ControlStop
```
Header header
bool stop_robot
```

### amr_msgs/KeyPressed
```
string key
```

---

## Sincronización de Tópicos

### MessageFilters - ApproximateTimeSynchronizer
```
Usado en: ParticleFilterNode
Sincroniza: /odometry y /scan

Parámetros:
- Slop time: 0.1 segundos
- Política: Por timestamp de header
- Alinea automáticamente mensajes correlacionados

Beneficios:
- Datos consistentes temporalmente
- Maneja diferencias de frecuencia
- Robusto a retrasos de transmisión
```

---

## Máquina de Estados (Lifecycle)

```
ESTADOS:
1. unconfigured  → inactive  → active  → finalized
                  (configure) (activate) (cleanup)

TRANSICIONES:
- Iniciación: unconfigured → inactive
- Activación: inactive → active
- Finalización: active → finalized → unconfigured

SERVICIOS ROS:
- /node_name/change_state (lifecycle_msgs/ChangeState)
- /node_name/get_state (lifecycle_msgs/GetState)

ORQUESTACIÓN:
LifecycleManager coordina el arranque en orden:
1. ParticleFilterNode (localización primero)
2. PRMNode (requiere pose de PF)
3. PurePursuitNode (requiere path de PRM)
4. WallFollowerNode (alternativa a PP)
5. CoppeliaSimNode / TurtleBot3 Interface (último)
```

---

## Errores y Consideraciones

### Errores Detectados

⚠️ **amr_msgs/KeyPressed (tb3_ws)**
- El mensaje está **importado pero no definido** en CMakeLists.txt
- **Solución**: Agregar a `tb3_ws/amr_msgs/CMakeLists.txt`:
  ```cmake
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/KeyPressed.msg"
  )
  ```

### Consideraciones de Implementación

1. **Sincronización**: Los nodos de control que usan `/odometry` y `/scan` DEBEN usarlos sincronizados
2. **Frecuencias**: CoppeliaSimNode @ 50Hz, ParticleFilter @ 10Hz max (procesamiento costoso)
3. **TwistStamped vs Twist**: Simulación usa TwistStamped (con timestamp), robot real usa Twist nativo
4. **Parámetros**: Deben ajustarse según ambiente (tamaño mapa, ruido sensores, etc.)
5. **Lifecycle**: Respetarprotocolo. No iniciar nodos sin coordinación

---

## Mejoras Futuras Posibles

- ✨ Integración con SLAM (cartographer/slam_toolbox)
- ✨ Mapas precargados en lugar de generación on-the-fly
- ✨ Control adaptativo con ajuste automático de parámetros
- ✨ Visión por computadora para validación (OpenCV)
- ✨ Planificación de trayectorias suave (Bézier/splines)
- ✨ Comunicación remota y logging centralizado
- ✨ Integración de objetivos dinámicos (no solo meta fija)
- ✨ Evasión de obstáculos dinámicos (más allá de wall-following)

---

## Conclusiones

Este sistema implementa un completo pipeline de navegación autónoma, escalable de simulación a robot real:

1. **P1**: Fundamentos de comunicación ROS 2
2. **P2**: Percepción y localización robusta
3. **P3**: Planificación y control deliberativo
4. **P4**: Integración con hardware real y teleopeación

Cada práctica **construye sobre la anterior**, manteniendo compatibilidad de interfaces pero aumentando sofisticación. El diseño modular permite reutilización de código y fácil extensión futura.
