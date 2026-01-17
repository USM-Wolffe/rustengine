//! Módulo de bindings Lua para exponer funciones Rust a la estrategia Lua
//! 
//! Este módulo expone todas las funciones necesarias para que la estrategia Lua
//! pueda interactuar con el RustEngine, incluyendo:
//! - Acceso a World (robots y balón)
//! - Funciones de Motion (move_to, move_direct, face_to, etc.)
//! - Funciones de GameController (get_ref_message)
//! - Funciones de Radio (send_velocity, kickx, kickz, dribbler)
//!
//! El modelo es imperativo: la estrategia llama funciones que ejecutan comandos
//! directamente, no retorna comandos para ejecutar después.

use mlua::{Lua, Result as LuaResult, Table, Function};
use std::sync::{Arc, RwLock as StdRwLock, Mutex as StdMutex};
use tokio::sync::{RwLock as TokioRwLock, Mutex as TokioMutex, mpsc};
use glam::Vec2;
use crate::world::{World, RobotState, BallState};
use crate::motion::{Motion, MotionCommand, KickerCommand};
use crate::game_controller::GameState;
use crate::protos::ssl_gc_referee_message::referee::Command;
use tracing::{error, warn, info};

// --- Comandos para enviar desde Lua a Rust ---

#[derive(Debug, Clone)]
pub enum LuaCommand {
    Motion(MotionCommand),
    Kicker(KickerCommand),
    // Requests que necesitan procesamiento con World completo y Motion
    MoveToRequest {
        id: i32,
        team: i32,
        target: glam::Vec2,
    },
    MoveDirectRequest {
        id: i32,
        team: i32,
        target: glam::Vec2,
    },
    FaceToRequest {
        id: i32,
        team: i32,
        target: glam::Vec2,
        kp: f64,
        ki: f64,
        kd: f64,
    },
}

// --- WorldSnapshot: Snapshot inmutable de World para pasar a Lua ---

/// Snapshot inmutable del estado del mundo para pasar a Lua
/// Evita tener que mantener locks durante la ejecución de Lua
#[derive(Debug, Clone)]
pub struct WorldSnapshot {
    pub ball: BallSnapshot,
    pub blue_robots: Vec<RobotSnapshot>,
    pub yellow_robots: Vec<RobotSnapshot>,
}

#[derive(Debug, Clone)]
pub struct RobotSnapshot {
    pub id: i32,
    pub team: i32,
    pub x: f64,
    pub y: f64,
    pub vel_x: f64,
    pub vel_y: f64,
    pub orientation: f64,
    pub active: bool,
}

#[derive(Debug, Clone)]
pub struct BallSnapshot {
    pub x: f64,
    pub y: f64,
    pub vel_x: f64,
    pub vel_y: f64,
}

impl From<&RobotState> for RobotSnapshot {
    fn from(robot: &RobotState) -> Self {
        Self {
            id: robot.id,
            team: robot.team,
            x: robot.position.x as f64,
            y: robot.position.y as f64,
            vel_x: robot.velocity.x as f64,
            vel_y: robot.velocity.y as f64,
            orientation: robot.orientation,
            active: robot.active,
        }
    }
}

impl From<&BallState> for BallSnapshot {
    fn from(ball: &BallState) -> Self {
        Self {
            x: ball.position.x as f64,
            y: ball.position.y as f64,
            vel_x: ball.velocity.x as f64,
            vel_y: ball.velocity.y as f64,
        }
    }
}

impl WorldSnapshot {
    /// Crea un snapshot del World actual (adquiere lock, copia datos, libera lock)
    pub async fn from_world(world: &Arc<TokioRwLock<World>>) -> Self {
        let world_guard = world.read().await;
        
        let ball = BallSnapshot::from(world_guard.get_ball_state());
        
        let blue_robots: Vec<RobotSnapshot> = world_guard.get_blue_team_active()
            .iter()
            .map(|r| RobotSnapshot::from(*r))
            .collect();
        
        let yellow_robots: Vec<RobotSnapshot> = world_guard.get_yellow_team_active()
            .iter()
            .map(|r| RobotSnapshot::from(*r))
            .collect();
        
        Self {
            ball,
            blue_robots,
            yellow_robots,
        }
    }
    
    /// Obtiene el estado de un robot específico
    pub fn get_robot_state(&self, id: i32, team: i32) -> Option<&RobotSnapshot> {
        match team {
            0 => self.blue_robots.iter().find(|r| r.id == id),
            1 => self.yellow_robots.iter().find(|r| r.id == id),
            _ => None,
        }
    }
}

// --- GameStateSnapshot: Snapshot inmutable de GameState para pasar a Lua ---

/// Snapshot inmutable del GameState para pasar a Lua
/// Evita tener que mantener locks durante la ejecución de Lua
#[derive(Debug, Clone)]
pub struct GameStateSnapshot {
    pub command: String,  // Comando como string ("HALT", "STOP", etc.)
}

/// Convierte referee::Command a string
fn command_to_string(cmd: crate::protos::ssl_gc_referee_message::referee::Command) -> String {
    match cmd {
        crate::protos::ssl_gc_referee_message::referee::Command::HALT => "HALT",
        crate::protos::ssl_gc_referee_message::referee::Command::STOP => "STOP",
        crate::protos::ssl_gc_referee_message::referee::Command::NORMAL_START => "NORMAL_START",
        crate::protos::ssl_gc_referee_message::referee::Command::FORCE_START => "FORCE_START",
        crate::protos::ssl_gc_referee_message::referee::Command::PREPARE_KICKOFF_YELLOW => "PREPARE_KICKOFF_YELLOW",
        crate::protos::ssl_gc_referee_message::referee::Command::PREPARE_KICKOFF_BLUE => "PREPARE_KICKOFF_BLUE",
        crate::protos::ssl_gc_referee_message::referee::Command::PREPARE_PENALTY_YELLOW => "PREPARE_PENALTY_YELLOW",
        crate::protos::ssl_gc_referee_message::referee::Command::PREPARE_PENALTY_BLUE => "PREPARE_PENALTY_BLUE",
        crate::protos::ssl_gc_referee_message::referee::Command::DIRECT_FREE_YELLOW => "DIRECT_FREE_YELLOW",
        crate::protos::ssl_gc_referee_message::referee::Command::DIRECT_FREE_BLUE => "DIRECT_FREE_BLUE",
        crate::protos::ssl_gc_referee_message::referee::Command::INDIRECT_FREE_YELLOW => "INDIRECT_FREE_YELLOW",
        crate::protos::ssl_gc_referee_message::referee::Command::INDIRECT_FREE_BLUE => "INDIRECT_FREE_BLUE",
        crate::protos::ssl_gc_referee_message::referee::Command::TIMEOUT_YELLOW => "TIMEOUT_YELLOW",
        crate::protos::ssl_gc_referee_message::referee::Command::TIMEOUT_BLUE => "TIMEOUT_BLUE",
        crate::protos::ssl_gc_referee_message::referee::Command::GOAL_YELLOW => "GOAL_YELLOW",
        crate::protos::ssl_gc_referee_message::referee::Command::GOAL_BLUE => "GOAL_BLUE",
        crate::protos::ssl_gc_referee_message::referee::Command::BALL_PLACEMENT_YELLOW => "BALL_PLACEMENT_YELLOW",
        crate::protos::ssl_gc_referee_message::referee::Command::BALL_PLACEMENT_BLUE => "BALL_PLACEMENT_BLUE",
        _ => "UNKNOWN",
    }.to_string()
}

// --- Contexto para las funciones Lua ---

/// Contexto compartido entre funciones Lua y Rust
/// Contiene referencias a World, Motion, GameState, Radio
/// Nota: No contiene Lua directamente para evitar problemas de Send/Sync
pub struct LuaContext {
    pub world: Arc<TokioRwLock<World>>,
    pub motion: Arc<TokioMutex<Motion>>,
    pub game_state: Arc<TokioRwLock<GameState>>,
    pub radio: Arc<TokioMutex<crate::radio::Radio>>,
    pub world_snapshot: Arc<StdRwLock<WorldSnapshot>>, // Usar std::sync::RwLock para acceso síncrono desde Lua
    pub game_state_snapshot: Arc<StdRwLock<GameStateSnapshot>>, // Usar std::sync::RwLock para acceso síncrono desde Lua
    pub command_tx: mpsc::UnboundedSender<LuaCommand>, // Canal para enviar comandos desde Lua a Rust
}

impl LuaContext {
    pub fn new(
        world: Arc<TokioRwLock<World>>,
        motion: Arc<TokioMutex<Motion>>,
        game_state: Arc<TokioRwLock<GameState>>,
        radio: Arc<TokioMutex<crate::radio::Radio>>,
        command_tx: mpsc::UnboundedSender<LuaCommand>,
    ) -> Self {
        let world_snapshot = Arc::new(StdRwLock::new(WorldSnapshot {
            ball: BallSnapshot { x: 0.0, y: 0.0, vel_x: 0.0, vel_y: 0.0 },
            blue_robots: Vec::new(),
            yellow_robots: Vec::new(),
        }));
        
        let game_state_snapshot = Arc::new(StdRwLock::new(GameStateSnapshot {
            command: "UNKNOWN".to_string(),
        }));
        
        Self {
            world,
            motion,
            game_state,
            radio,
            world_snapshot,
            game_state_snapshot,
            command_tx,
        }
    }
    
    /// Actualiza el snapshot del mundo (debe llamarse antes de ejecutar process())
    pub async fn update_snapshot(&self) {
        let snapshot = WorldSnapshot::from_world(&self.world).await;
        *self.world_snapshot.write().unwrap() = snapshot;
        self.update_game_state_snapshot().await;
    }
    
    /// Actualiza el snapshot del GameState (llamar junto con update_snapshot)
    pub async fn update_game_state_snapshot(&self) {
        let game_state_guard = self.game_state.read().await;
        let command_str = command_to_string(game_state_guard.command);
        
        let mut snapshot_guard = self.game_state_snapshot.write().unwrap();
        snapshot_guard.command = command_str;
    }
}

// --- Funciones para exponer a Lua ---

/// Configura el entorno Lua con todas las funciones expuestas desde Rust
/// Usa funciones síncronas que envían comandos a un canal
pub fn setup_lua_environment(
    lua: &Lua,
    ctx: Arc<LuaContext>,
) -> LuaResult<()> {
    // Crear tabla global "Engine" para todas las funciones nativas
    let engine = lua.create_table()?;
    
    // --- Funciones de World (síncronas, leen del snapshot) ---
    
    // get_robot_state(robotId: number, team: number) -> RobotState | nil
    let get_robot_state = {
        let ctx = ctx.clone();
        lua.create_function(move |lua, (id, team): (i32, i32)| {
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                return Ok(None::<Table>);
            }
            
            // Obtener snapshot (acceso síncrono con std::sync::RwLock)
            let snapshot_guard = ctx.world_snapshot.read().unwrap();
            if let Some(robot) = snapshot_guard.get_robot_state(id, team) {
                let table = lua.create_table()?;
                table.set("id", robot.id)?;
                table.set("team", robot.team)?;
                table.set("x", robot.x)?;
                table.set("y", robot.y)?;
                table.set("vel_x", robot.vel_x)?;
                table.set("vel_y", robot.vel_y)?;
                table.set("orientation", robot.orientation)?;
                table.set("active", robot.active)?;
                Ok(Some(table))
            } else {
                Ok(None::<Table>)
            }
        })?
    };
    let get_robot_state_global = get_robot_state.clone();
    engine.set("get_robot_state", get_robot_state)?;
    lua.globals().set("get_robot_state", get_robot_state_global)?;
    
    // get_ball_state() -> BallState
    let get_ball_state = {
        let ctx = ctx.clone();
        lua.create_function(move |lua, ()| {
            let snapshot_guard = ctx.world_snapshot.read().unwrap();
            let ball = &snapshot_guard.ball;
            let table = lua.create_table()?;
            table.set("x", ball.x)?;
            table.set("y", ball.y)?;
            table.set("vel_x", ball.vel_x)?;
            table.set("vel_y", ball.vel_y)?;
            Ok(table)
        })?
    };
    let get_ball_state_global = get_ball_state.clone();
    engine.set("get_ball_state", get_ball_state)?;
    lua.globals().set("get_ball_state", get_ball_state_global)?;
    
    // get_blue_team_state() -> RobotState[]
    let get_blue_team_state = {
        let ctx = ctx.clone();
        lua.create_function(move |lua, ()| {
            let snapshot_guard = ctx.world_snapshot.read().unwrap();
            let mut robots = lua.create_table()?;
            for (i, robot) in snapshot_guard.blue_robots.iter().enumerate() {
                let robot_table = lua.create_table()?;
                robot_table.set("id", robot.id)?;
                robot_table.set("team", robot.team)?;
                robot_table.set("x", robot.x)?;
                robot_table.set("y", robot.y)?;
                robot_table.set("vel_x", robot.vel_x)?;
                robot_table.set("vel_y", robot.vel_y)?;
                robot_table.set("orientation", robot.orientation)?;
                robot_table.set("active", robot.active)?;
                robots.set(i + 1, robot_table)?; // Lua arrays empiezan en 1
            }
            Ok(robots)
        })?
    };
    let get_blue_team_state_global = get_blue_team_state.clone();
    engine.set("get_blue_team_state", get_blue_team_state)?;
    lua.globals().set("get_blue_team_state", get_blue_team_state_global)?;
    
    // get_yellow_team_state() -> RobotState[]
    let get_yellow_team_state = {
        let ctx = ctx.clone();
        lua.create_function(move |lua, ()| {
            let snapshot_guard = ctx.world_snapshot.read().unwrap();
            let mut robots = lua.create_table()?;
            for (i, robot) in snapshot_guard.yellow_robots.iter().enumerate() {
                let robot_table = lua.create_table()?;
                robot_table.set("id", robot.id)?;
                robot_table.set("team", robot.team)?;
                robot_table.set("x", robot.x)?;
                robot_table.set("y", robot.y)?;
                robot_table.set("vel_x", robot.vel_x)?;
                robot_table.set("vel_y", robot.vel_y)?;
                robot_table.set("orientation", robot.orientation)?;
                robot_table.set("active", robot.active)?;
                robots.set(i + 1, robot_table)?; // Lua arrays empiezan en 1
            }
            Ok(robots)
        })?
    };
    let get_yellow_team_state_global = get_yellow_team_state.clone();
    engine.set("get_yellow_team_state", get_yellow_team_state)?;
    lua.globals().set("get_yellow_team_state", get_yellow_team_state_global)?;
    
    // --- Funciones de Motion (síncronas, envían comandos al canal) ---
    
    // move_to(robotId: number, team: number, point: {x: number, y: number})
    // Envía un MoveToRequest que será procesado por el task async con path planning completo
    let move_to = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, (id, team, point_table): (i32, i32, Table)| {
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                warn!("move_to: Parámetros inválidos (id={}, team={})", id, team);
                return Ok(());
            }
            
            // Leer punto
            let x: f64 = point_table.get("x").ok().unwrap_or(0.0);
            let y: f64 = point_table.get("y").ok().unwrap_or(0.0);
            let target = glam::Vec2::new(x as f32, y as f32);
            
            // Enviar request al canal (será procesado por task async con World completo y path planning)
            if let Err(e) = ctx.command_tx.send(LuaCommand::MoveToRequest {
                id,
                team,
                target,
            }) {
                warn!("move_to: Error enviando request: {}", e);
            }
            
            Ok(())
        })?
    };
    let move_to_global = move_to.clone();
    engine.set("move_to", move_to)?;
    lua.globals().set("move_to", move_to_global)?;
    
    // move_direct(robotId: number, team: number, point: {x: number, y: number})
    // Envía un MoveDirectRequest que será procesado por el task async sin path planning
    let move_direct = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, (id, team, point_table): (i32, i32, Table)| {
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                warn!("move_direct: Parámetros inválidos (id={}, team={})", id, team);
                return Ok(());
            }
            
            // Leer punto
            let x: f64 = point_table.get("x").ok().unwrap_or(0.0);
            let y: f64 = point_table.get("y").ok().unwrap_or(0.0);
            let target = glam::Vec2::new(x as f32, y as f32);
            
            // Enviar request al canal (será procesado por task async sin path planning)
            if let Err(e) = ctx.command_tx.send(LuaCommand::MoveDirectRequest {
                id,
                team,
                target,
            }) {
                warn!("move_direct: Error enviando request: {}", e);
            }
            
            Ok(())
        })?
    };
    let move_direct_global = move_direct.clone();
    engine.set("move_direct", move_direct)?;
    lua.globals().set("move_direct", move_direct_global)?;
    
    // face_to(robotId: number, team: number, point: {x: number, y: number}, kp?: number, ki?: number, kd?: number)
    // Envía un FaceToRequest que será procesado por el task async con PID completo y estado persistente
    let face_to = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, (id, team, point_table, kp, ki, kd): (i32, i32, Table, Option<f64>, Option<f64>, Option<f64>)| {
            let kp = kp.unwrap_or(3.5);
            let ki = ki.unwrap_or(0.7);
            let kd = kd.unwrap_or(0.1);
            
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                warn!("face_to: Parámetros inválidos (id={}, team={})", id, team);
                return Ok(());
            }
            
            // Leer punto
            let x: f64 = point_table.get("x").ok().unwrap_or(0.0);
            let y: f64 = point_table.get("y").ok().unwrap_or(0.0);
            let target = glam::Vec2::new(x as f32, y as f32);
            
            // Enviar request al canal (será procesado por task async con PID completo y estado persistente)
            if let Err(e) = ctx.command_tx.send(LuaCommand::FaceToRequest {
                id,
                team,
                target,
                kp,
                ki,
                kd,
            }) {
                warn!("face_to: Error enviando request: {}", e);
            }
            
            Ok(())
        })?
    };
    let face_to_global = face_to.clone();
    engine.set("face_to", face_to)?;
    lua.globals().set("face_to", face_to_global)?;
    
    // send_velocity(robotId: number, team: number, vx: number, vy: number, vtetha: number)
    let send_velocity = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, (id, team, vx, vy, vtetha): (i32, i32, f64, f64, f64)| {
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                warn!("send_velocity: Parámetros inválidos (id={}, team={})", id, team);
                return Ok(());
            }
            
            // Validar límites de velocidad
            let vx = vx.clamp(-3.0, 3.0);
            let vy = vy.clamp(-3.0, 3.0);
            let vtetha = vtetha.clamp(-10.0, 10.0);
            
            // Obtener orientación del robot desde snapshot
            let orientation = {
                let snapshot_guard = ctx.world_snapshot.read().unwrap();
                snapshot_guard.get_robot_state(id, team)
                    .map(|r| r.orientation)
                    .unwrap_or(0.0)
            };
            
            // Crear comando de movimiento directo
            let motion_cmd = MotionCommand {
                id,
                team,
                vx,
                vy,
                omega: vtetha,
                orientation,
            };
            
            // Enviar comando al canal
            if let Err(e) = ctx.command_tx.send(LuaCommand::Motion(motion_cmd)) {
                warn!("send_velocity: Error enviando comando: {}", e);
            }
            
            Ok(())
        })?
    };
    let send_velocity_global = send_velocity.clone();
    engine.set("send_velocity", send_velocity)?;
    lua.globals().set("send_velocity", send_velocity_global)?;
    
    // --- Funciones de Kicker ---
    
    // kickx(robotId: number, team: number)
    let kickx = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, (id, team): (i32, i32)| {
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                warn!("kickx: Parámetros inválidos (id={}, team={})", id, team);
                return Ok(());
            }
            
            // Crear comando de kicker (kick_x = true)
            let kicker_cmd = KickerCommand {
                id,
                team,
                kick_x: true,
                kick_z: false,
                dribbler: 0.0,
            };
            
            // Enviar comando al canal
            if let Err(e) = ctx.command_tx.send(LuaCommand::Kicker(kicker_cmd)) {
                warn!("kickx: Error enviando comando: {}", e);
            }
            
            Ok(())
        })?
    };
    let kickx_global = kickx.clone();
    engine.set("kickx", kickx)?;
    lua.globals().set("kickx", kickx_global)?;
    
    // kickz(robotId: number, team: number)
    let kickz = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, (id, team): (i32, i32)| {
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                warn!("kickz: Parámetros inválidos (id={}, team={})", id, team);
                return Ok(());
            }
            
            // Crear comando de kicker (kick_z = true)
            let kicker_cmd = KickerCommand {
                id,
                team,
                kick_x: false,
                kick_z: true,
                dribbler: 0.0,
            };
            
            // Enviar comando al canal
            if let Err(e) = ctx.command_tx.send(LuaCommand::Kicker(kicker_cmd)) {
                warn!("kickz: Error enviando comando: {}", e);
            }
            
            Ok(())
        })?
    };
    let kickz_global = kickz.clone();
    engine.set("kickz", kickz)?;
    lua.globals().set("kickz", kickz_global)?;
    
    // dribbler(robotId: number, team: number, speed: number)
    let dribbler = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, (id, team, speed): (i32, i32, f64)| {
            // Validar parámetros
            if id < 0 || id > 10 || (team != 0 && team != 1) {
                warn!("dribbler: Parámetros inválidos (id={}, team={})", id, team);
                return Ok(());
            }
            
            // Limitar velocidad del dribbler (0-4 según documentación del Engine C++)
            let speed = speed.clamp(0.0, 4.0);
            
            // Crear comando de kicker (solo dribbler)
            let kicker_cmd = KickerCommand {
                id,
                team,
                kick_x: false,
                kick_z: false,
                dribbler: speed,
            };
            
            // Enviar comando al canal
            if let Err(e) = ctx.command_tx.send(LuaCommand::Kicker(kicker_cmd)) {
                warn!("dribbler: Error enviando comando: {}", e);
            }
            
            Ok(())
        })?
    };
    let dribbler_global = dribbler.clone();
    engine.set("dribbler", dribbler)?;
    lua.globals().set("dribbler", dribbler_global)?;
    
    // --- Funciones de GameController ---
    
    // get_ref_message() -> string
    // Retorna el comando actual del referee desde el snapshot síncrono
    let get_ref_message = {
        let ctx = ctx.clone();
        lua.create_function(move |_lua, ()| {
            let snapshot_guard = ctx.game_state_snapshot.read().unwrap();
            Ok(snapshot_guard.command.clone())
        })?
    };
    let get_ref_message_global = get_ref_message.clone();
    engine.set("get_ref_message", get_ref_message)?;
    lua.globals().set("get_ref_message", get_ref_message_global)?;
    
    // Registrar tabla global "Engine"
    lua.globals().set("Engine", engine)?;
    
    Ok(())
}

/// Ejecuta la función process() desde Lua usando un thread dedicado
/// Nota: Este es un workaround porque mlua::Lua no es Send
pub fn execute_process_sync(
    lua: &StdMutex<Lua>,
) -> LuaResult<()> {
    let mut lua_guard = lua.lock().unwrap();
    let process: Function = lua_guard.globals().get("process")?;
    process.call::<(), ()>(())?;
    Ok(())
}

/// Carga y configura un script Lua de estrategia
/// Nota: Debe ejecutarse en el mismo thread donde se usa Lua
pub fn load_strategy_sync(
    lua: &mut Lua,
    ctx: Arc<LuaContext>,
    script_path: &str,
) -> LuaResult<()> {
    // Obtener el directorio del script para configurar package.path
    let script_dir = std::path::Path::new(script_path)
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .to_string_lossy()
        .to_string();
    
    // Configurar package.path para que Lua pueda encontrar módulos en el directorio del script
    // Usamos formato Windows y Unix para compatibilidad
    let lua_path = format!(
        "{}\\?.lua;{}\\?\\init.lua;{}/?.lua;{}/?/init.lua;",
        script_dir, script_dir, script_dir, script_dir
    );
    
    // Obtener package.path actual y agregarlo al nuevo path
    let package: Table = lua.globals().get("package")?;
    let current_path: String = package.get("path").unwrap_or_else(|_| ";".to_string());
    let new_path = format!("{}{}", lua_path, current_path);
    package.set("path", new_path)?;
    
    info!("[Lua] package.path configurado para buscar módulos en: {}", script_dir);
    
    // Leer script
    let script = std::fs::read_to_string(script_path)
        .map_err(|e| mlua::Error::ExternalError(Arc::new(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("No se pudo leer el script: {}", e),
        ))))?;
    
    // Configurar entorno
    setup_lua_environment(lua, ctx)?;
    
    // Cargar y ejecutar script
    lua.load(&script).exec()?;
    
    info!("[Lua] Script de estrategia cargado desde: {}", script_path);
    
    Ok(())
}
