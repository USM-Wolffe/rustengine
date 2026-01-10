mod protos;
mod vision;
mod tracker;
mod world;
mod motion;
mod radio;
mod game_controller;
#[path = "GUI/mod.rs"]
mod gui;

use tokio::sync::{mpsc, Mutex as TokioMutex, RwLock as TokioRwLock};
use vision::{Vision, VisionEvent};
use gui::ConfigUpdate;
use world::{World, RobotState};
use game_controller::{GameController, GameState};
use std::time::Duration;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

fn main() {
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
                Err(_e) => {
                    // Skip motion/radio integration if radio creation fails
                    // Keep the runtime alive
                    loop {
                        tokio::time::sleep(Duration::from_secs(1)).await;
                    }
                }
            };
            
            // Spawn a task to periodically compute motion commands and send them
            // Note: This uses blocking reads from RwLock. For better async performance,
            // consider migrating World to use tokio::sync::RwLock in the future.
            let world_for_motion = world_clone.clone();
            let radio_for_motion = radio.clone();
            let motion_for_loop = motion.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_millis(16)); // ~60 FPS
                loop {
                    interval.tick().await;
                    
                    // Read world state (async read)
                    let (ball_pos, robots_data): (glam::Vec2, Vec<RobotState>) = {
                        let world = world_for_motion.read().await;
                        let ball_pos = world.get_ball_state().position;
                        let robots_data: Vec<_> = world.get_blue_team_active()
                            .into_iter()
                            .cloned()
                            .collect();
                        (ball_pos, robots_data)
                    };
                    
                    if robots_data.is_empty() {
                        continue;
                    }
                    
                    let mut radio_guard = radio_for_motion.lock().await;
                    let mut motion_guard = motion_for_loop.lock().await;
                    
                    let mut commands_added = 0;
                    for robot_state in robots_data {
                        if robot_state.active {
                            let motion_cmd = {
                                let world = world_for_motion.read().await;
                                motion_guard.move_to(&robot_state, ball_pos, &world)
                            };
                            radio_guard.add_motion_command(motion_cmd);
                            commands_added += 1;
                        }
                    }
                    
                    if commands_added > 0 {
                        let _ = radio_guard.send_commands().await;
                    }
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