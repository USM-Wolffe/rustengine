mod protos;
mod vision;
mod tracker;
mod world;
mod motion;
mod radio;
mod game_controller;
mod lua_bindings;
#[path = "GUI/mod.rs"]
mod gui;

use tokio::sync::{mpsc, Mutex as TokioMutex, RwLock as TokioRwLock};
use vision::{Vision, VisionEvent};
use gui::ConfigUpdate;
use world::World;
use game_controller::{GameController, GameState};
use lua_bindings::{LuaContext, load_strategy_sync, execute_process_sync};
use mlua::Lua;
use std::time::Duration;
use std::sync::{Arc, Mutex as StdMutex, atomic::{AtomicBool, Ordering}};
use tracing::{error, warn, info};

fn main() {
    // Leer argumento de línea de comandos (ruta al script Lua)
    let script_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| {
            eprintln!("Uso: rustengine.exe <ruta_al_script_lua>");
            eprintln!("Ejemplo: rustengine.exe strategy/main.lua");
            eprintln!("Ejemplo: rustengine.exe C:\\ruta\\a\\strategy\\main.lua");
            std::process::exit(1);
        });
    
    info!("[RustEngine] Iniciando con estrategia Lua: {}", script_path);
    // Create channels
    let (vision_tx, mut vision_rx) = mpsc::channel(100); 
    let (status_tx, status_rx) = mpsc::channel(100);
    let (config_tx, mut config_rx) = mpsc::channel(10);

    let vision_ip = "224.5.23.2".to_string();
    let vision_port = 10020;
    
    // GameController multicast address (standard SSL port)
    let gc_ip = "224.5.23.1".to_string();
    let gc_port = 10003;
    
    let gui_ip = vision_ip.clone();
    let gui_port = vision_port;
    let gui_config_tx = config_tx.clone();

    // Create World instance (11 robots per team for SSL)
    // Usar TokioRwLock para acceso async-friendly
    let world = Arc::new(TokioRwLock::new(World::new(11, 11)));
    let world_clone = world.clone();
    
    // Create GameController state (shared across modules)
    let game_state = Arc::new(TokioRwLock::new(GameState::default()));
    let game_state_clone = game_state.clone();
    
    // Create shared flag for tracker state
    let tracker_enabled = Arc::new(AtomicBool::new(true)); // Habilitado por defecto
    let tracker_enabled_clone = tracker_enabled.clone();

    // Spawn a background thread to run the vision system
    std::thread::spawn(move || {
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            // Spawn a task to handle configuration updates and restart vision
            tokio::spawn(async move {
                let mut current_ip = vision_ip;
                let mut current_port = vision_port;
                let tracker_enabled = tracker_enabled_clone;
                
                loop {
                    let mut vision_system = Vision::new(current_ip.clone(), current_port, tracker_enabled.clone());
                    let vision_tx_clone = vision_tx.clone();
                    let status_tx_clone = status_tx.clone();
                    
                    // Spawn vision task
                    let mut vision_handle = tokio::spawn(async move {
                        let _ = vision_system.run(vision_tx_clone, status_tx_clone).await;
                    });

                    // Wait for config update or vision task to complete
                    tokio::select! {
                        Some(config) = config_rx.recv() => {
                            // Update configuration
                            match config {
                                ConfigUpdate::ChangeIpPort(new_ip, new_port) => {
                                    current_ip = new_ip;
                                    current_port = new_port;
                                    // Abort and loop will restart with new config
                                    if !vision_handle.is_finished() {
                                        vision_handle.abort();
                                    }
                                }
                                ConfigUpdate::ToggleTracker(enabled) => {
                                    tracker_enabled.store(enabled, Ordering::Relaxed);
                                    // No reiniciar el task, el flag se lee en tiempo real
                                    continue; // Continuar el loop sin reiniciar vision
                                }
                            }
                        }
                        _result = &mut vision_handle => {
                            // Vision task completed unexpectedly
                            // Add a small delay before restarting to prevent tight loop
                            tokio::time::sleep(Duration::from_millis(100)).await;
                        }
                    }
                    
                    // If we're here from config update, abort the vision task
                    if !vision_handle.is_finished() {
                        vision_handle.abort();
                    }
                }
            });

            // Spawn a background task to consume vision events and update World
            let world_for_vision = world_clone.clone();
            tokio::spawn(async move {
                while let Some(event) = vision_rx.recv().await {
                    let mut world = world_for_vision.write().await;
                    
                    match event {
                        VisionEvent::Robot(robot_data) => {
                            world.update_robot(
                                robot_data.id as i32,
                                robot_data.team as i32,
                                robot_data.position,
                                robot_data.orientation as f64,
                                robot_data.velocity,
                                robot_data.angular_velocity as f64,
                            );
                        }
                        VisionEvent::Ball(ball_data) => {
                            world.update_ball(ball_data.position, ball_data.velocity);
                        }
                    }
                }
            });

            // Spawn a task to periodically update World (mark inactive robots)
            let world_for_update = world_clone.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_millis(100));
                loop {
                    interval.tick().await;
                    let mut world = world_for_update.write().await;
                    world.update();
                }
            });

            // Spawn GameController task
            let gc_ip_clone = gc_ip.clone();
            let gc_state_for_gc = game_state_clone.clone();
            tokio::spawn(async move {
                let mut gc = GameController::new(gc_ip_clone, gc_port);
                // Run GameController (this will loop forever)
                if let Err(e) = gc.run(gc_state_for_gc, None).await {
                    eprintln!("[GameController] Error fatal: {}", e);
                }
            });

            // Initialize Motion and Radio systems
            let motion = Arc::new(TokioMutex::new(motion::Motion::new()));
            let radio = match radio::Radio::new(false, "127.0.0.1", 20011).await {
                Ok(r) => Arc::new(TokioMutex::new(r)),
                Err(e) => {
                    error!("[Radio] Error al crear Radio: {}", e);
                    eprintln!("[Radio] Error al crear Radio: {}", e);
                    // Skip motion/radio integration if radio creation fails
                    // Keep the runtime alive
                    loop {
                        tokio::time::sleep(Duration::from_secs(1)).await;
                    }
                }
            };
            
            // Crear canal para comandos desde Lua (unbounded para no bloquear)
            let (command_tx, mut command_rx) = mpsc::unbounded_channel();
            
            // Crear contexto Lua (sin incluir Lua directamente)
            let lua_ctx = Arc::new(LuaContext::new(
                world_clone.clone(),
                motion.clone(),
                game_state_clone.clone(),
                radio.clone(),
                command_tx.clone(),
            ));
            
            // Crear thread dedicado para Lua (porque mlua::Lua no es Send)
            // Este thread necesita un runtime de Tokio para poder llamar a funciones async
            let script_path_for_thread = script_path.clone();
            let lua_ctx_for_thread = lua_ctx.clone();
            
            // Canal para sincronizar la carga del script
            let (load_tx, mut load_rx) = mpsc::channel::<Result<(), String>>(1);
            
            std::thread::spawn(move || {
                // Crear un runtime local para este thread (ya que no podemos usar el runtime principal)
                let local_rt = tokio::runtime::Runtime::new().unwrap();
                
                // Crear Lua en este thread dedicado
                // Lua::new() devuelve Lua directamente (no Result) con las features luajit/vendored
                let mut lua = Lua::new();
                
                // Cargar script (necesitamos mutable, así que lo hacemos antes de meterlo en el Mutex)
                match load_strategy_sync(&mut lua, lua_ctx_for_thread.clone(), &script_path_for_thread) {
                    Ok(_) => {
                        let _ = load_tx.blocking_send(Ok(()));
                    }
                    Err(e) => {
                        let _ = load_tx.blocking_send(Err(format!("Error cargando script: {}", e)));
                        return;
                    }
                }
                
                // Mantener Lua en un Mutex para acceso seguro (ahora que ya está configurado)
                let lua_mutex = StdMutex::new(lua);
                
                // Loop para ejecutar process() periódicamente (~60 FPS)
                let mut last_update = std::time::Instant::now();
                loop {
                    let elapsed = last_update.elapsed();
                    if elapsed.as_millis() >= 16 {
                        // Actualizar snapshot antes de ejecutar process()
                        local_rt.block_on(async {
                            lua_ctx_for_thread.update_snapshot().await;
                        });
                        
                        // Ejecutar process()
                        if let Err(e) = execute_process_sync(&lua_mutex) {
                            error!("[Lua] Error ejecutando process(): {}", e);
                        }
                        
                        last_update = std::time::Instant::now();
                    }
                    
                    // Pequeña pausa para no saturar el CPU
                    std::thread::sleep(Duration::from_millis(1));
                }
            });
            
            // Esperar a que el script se cargue
            if let Some(result) = load_rx.recv().await {
                match result {
                    Ok(_) => {
                        info!("[Lua] Estrategia cargada correctamente desde: {}", script_path);
                    }
                    Err(e) => {
                        error!("[Lua] Error al cargar estrategia: {}", e);
                        eprintln!("[Lua] Error al cargar estrategia: {}", e);
                        eprintln!("[Lua] Asegúrate de que la ruta al script Lua sea correcta");
                        std::process::exit(1);
                    }
                }
            }
            
            // Spawn a task to process commands from Lua channel and add to Radio
            let radio_for_commands = radio.clone();
            tokio::spawn(async move {
                loop {
                    // Procesar comandos del canal (no bloqueante con try_recv)
                    while let Ok(cmd) = command_rx.try_recv() {
                        match cmd {
                            lua_bindings::LuaCommand::Motion(motion_cmd) => {
                                let mut radio_guard = radio_for_commands.lock().await;
                                radio_guard.add_motion_command(motion_cmd);
                            }
                            lua_bindings::LuaCommand::Kicker(kicker_cmd) => {
                                let mut radio_guard = radio_for_commands.lock().await;
                                radio_guard.add_kicker_command(kicker_cmd);
                            }
                        }
                    }
                    
                    // Enviar comandos acumulados en Radio a robots
                    let mut radio_guard = radio_for_commands.lock().await;
                    if let Err(e) = radio_guard.send_commands().await {
                        warn!("[Radio] Error enviando comandos: {}", e);
                    }
                    
                    tokio::time::sleep(Duration::from_millis(16)).await; // ~60 FPS
                }
            });

            // Keep the runtime alive
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        });
    });

    // Run GUI (blocks until window is closed)
    // The GUI will consume status_rx directly through its subscription
    let _ = gui::run_gui(gui_ip, gui_port, gui_config_tx, status_rx);
}