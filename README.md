# caddy_ai2_ros2_robot_lowlevel

Control de bajo nivel del **Caddy AI2** (carrito de golf, plataforma de pruebas de vehículos) en hardware real.
Corre en el **LattePanda Alpha**. El otro ordenador del vehículo, la Jetson Orin NX, se documenta en
[`caddy_ai2_ros2_robot_perception`](https://github.com/racarla96/caddy_ai2_ros2_robot_perception).

**ROS 2:** Jazzy | **Plataforma:** Ubuntu 24.04 | **Proyecto:** CERVAREC

El repo es un workspace: los paquetes van en `src/` como submódulos y `src/caddy_ai2_ros2_robot_lowlevel/`
contiene el launch y la configuración que los orquestan.

## Qué lleva

| Paquete | Rol |
|---|---|
| `caddy_ai2_ros2_description` | Descripción del robot (fuente única compartida con la simulación) |
| `caddy_ai2_ros2_common` | SocketCAN, utilidades de launch |
| `caddy_ai2_ros2_control_system_steering_driver` | Hardware interface de la dirección (CANopen CiA 402) |
| `caddy_ai2_ros2_control_system_traction_driver` | Hardware interface de la tracción (Curtis CAN) |
| `caddy_ai2_ros2_bicycle_to_ackermann_{steering,traction}_adapter` | Adaptadores cinemáticos |
| `caddy_ai2_ros2_robot_description_publisher` | Publicador de `robot_description` |
| `caddy_ai2_ros2_sensors_sick_lms_291` | LiDAR 2D SICK LMS 291 |
| `caddy_ai2_ros2_control_sensors_sbg_ig_500n` | IMU (y GPS) SBG IG-500N |

**Pendiente de integrar** (repos existentes, por revisar): `caddy_ai2_ros2_plc_schneider_tm241ce40t`,
`caddy_ai2_ros2_controllers`, `caddy_ai2_ros2_automatic_brake_system`.

## Clonar y compilar

```bash
git clone --recurse-submodules git@github.com:racarla96/caddy_ai2_ros2_robot_lowlevel.git
cd caddy_ai2_ros2_robot_lowlevel
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

El SICK necesita la librería nativa `sicklms` y el SBG el SDK `sbgCom` (sin él compila sin los targets de hardware).

## Lanzar

```bash
ros2 launch caddy_ai2_ros2_robot_lowlevel robot.launch.py
```

## Pendiente

- El launch recorre todos los sensores con `enabled: true` de `robot_params.yaml`; hay que filtrar por ordenador
  (campo `host: lattepanda|jetson`) para que aquí solo arranquen los de este equipo.
- Parámetros de calibración de la dirección (`counts_per_radian`, `abs_zero_position`, `gain_inc_to_abs`) a un fichero dedicado.
- Variante con y sin PLC (`use_plc`).

Plan completo: [`PLAN_REORGANIZACION.md`](https://github.com/racarla96/caddy_ai2/blob/jazzy/docs/PLAN_REORGANIZACION.md).
Licencia CC BY 4.0.
