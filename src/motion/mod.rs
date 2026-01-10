mod commands;
mod environment;
mod path_planner;
mod pid;
mod bang_bang;

pub use commands::{MotionCommand, KickerCommand, RobotCommand};
pub use environment::Environment;
pub use path_planner::FastPathPlanner;
pub use pid::PIDController;
pub use bang_bang::BangBangController;

use crate::world::{World, RobotState};
use glam::Vec2;

/// Módulo principal de control de movimiento
pub struct Motion {
    path_planner: FastPathPlanner,
    bangbang: BangBangController,
}

impl Motion {
    /// Crea un nuevo sistema de movimiento
    pub fn new() -> Self {
        Self {
            path_planner: FastPathPlanner::new(5), // max_depth = 5
            bangbang: BangBangController::new(15.0, 2.5), // max_accel=15.0 m/s² (muy alta), max_vel=2.5 m/s
        }
    }
    
    /// Normaliza un ángulo al rango [-π, π]
    /// Implementación consistente con tracker
    pub fn normalize_angle(angle: f64) -> f64 {
        let pi = std::f64::consts::PI;
        let two_pi = 2.0 * pi;
        let mut normalized = angle % two_pi;
        if normalized > pi {
            normalized -= two_pi;
        } else if normalized <= -pi {
            normalized += two_pi;
        }
        normalized
    }
    
    /// Movimiento hacia un objetivo usando path planner con Bang-Bang Control
    pub fn move_to(
        &mut self,
        robot_state: &RobotState,
        target: Vec2,
        world: &World,
    ) -> MotionCommand {
        let env = Environment::new(world, robot_state);
        let path = self.path_planner.get_path(robot_state.position, target, &env);
        
        // Calcular velocidad hacia el siguiente punto del path
        if path.len() > 1 {
            let next_point = path[1];
            let direction = (next_point - robot_state.position).normalize();
            let speed = 2.0; // Velocidad máxima (m/s) - aumentada para compensar Bang-Bang
            
            MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: direction.x as f64 * speed,
                vy: direction.y as f64 * speed,
                omega: 0.0,
                orientation: robot_state.orientation,
            }
        } else {
            // Ya está en el objetivo o path inválido
            MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: robot_state.orientation,
            }
        }
    }
    
    /// Movimiento directo sin evasión de obstáculos
    pub fn move_direct(
        &mut self,
        robot_state: &RobotState,
        target: Vec2,
    ) -> MotionCommand {
        let diff = target - robot_state.position;
        let distance = diff.length();
        
        // Si está muy cerca del objetivo, detenerse
        if distance < 0.1 {
            return MotionCommand {
                id: robot_state.id,
                team: robot_state.team,
                vx: 0.0,
                vy: 0.0,
                omega: 0.0,
                orientation: robot_state.orientation,
            };
        }
        
        let direction = diff.normalize();
        let speed = 2.0f32.min(distance * 2.0); // Velocidad proporcional, máximo 2.0 m/s
        let speed = speed.max(0.1); // Mínimo 0.1 m/s para que se mueva
        
        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: (direction.x * speed) as f64,
            vy: (direction.y * speed) as f64,
            omega: 0.0,
            orientation: robot_state.orientation,
        }
    }
    
    /// Control PID personalizado para movimiento en X e Y
    pub fn motion(
        &mut self,
        robot_state: &RobotState,
        target: Vec2,
        _world: &World,
        kp_x: f64,
        ki_x: f64,
        kp_y: f64,
        ki_y: f64,
    ) -> MotionCommand {
        let error = target - robot_state.position;
        
        // Controladores PID separados para X e Y
        let mut pid_x = PIDController::new(kp_x, ki_x, 0.0);
        let mut pid_y = PIDController::new(kp_y, ki_y, 0.0);
        
        let dt = 0.016; // ~60 FPS
        let vx = pid_x.compute(error.x as f64, dt);
        let vy = pid_y.compute(error.y as f64, dt);
        
        // Limitar velocidad máxima
        let max_speed = 1.5;
        let speed = (vx * vx + vy * vy).sqrt();
        let (vx_limited, vy_limited) = if speed > max_speed {
            let scale = max_speed / speed;
            (vx * scale, vy * scale)
        } else {
            (vx, vy)
        };
        
        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: vx_limited,
            vy: vy_limited,
            omega: 0.0,
            orientation: robot_state.orientation,
        }
    }
    
    /// Orientar hacia un punto
    pub fn face_to(
        &self,
        robot_state: &RobotState,
        target: Vec2,
        kp: f64,
        ki: f64,
        kd: f64,
    ) -> MotionCommand {
        let direction = (target - robot_state.position).normalize();
        let target_angle = direction.y.atan2(direction.x) as f64;
        self.face_to_angle(robot_state, target_angle, kp, ki, kd)
    }
    
    /// Orientar hacia un ángulo específico
    pub fn face_to_angle(
        &self,
        robot_state: &RobotState,
        target_angle: f64,
        kp: f64,
        ki: f64,
        kd: f64,
    ) -> MotionCommand {
        let mut pid = PIDController::new(kp, ki, kd);
        let error = Self::normalize_angle(target_angle - robot_state.orientation);
        let dt = 0.016; // ~60 FPS
        let omega = pid.compute(error, dt);
        
        // Limitar velocidad angular máxima
        let max_omega = 3.0; // rad/s
        let omega_limited = omega.max(-max_omega).min(max_omega);
        
        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: 0.0,
            vy: 0.0,
            omega: omega_limited,
            orientation: robot_state.orientation,
        }
    }
    
    /// Movimiento con orientación simultáneos
    pub fn motion_with_orientation(
        &mut self,
        robot_state: &RobotState,
        target: Vec2,
        target_angle: f64,
        world: &World,
        kp_x: f64,
        ki_x: f64,
        kp_y: f64,
        ki_y: f64,
        kp_theta: f64,
        ki_theta: f64,
        kd_theta: f64,
    ) -> MotionCommand {
        // Combinar move_to y face_to_angle
        let motion_cmd = self.move_to(robot_state, target, world);
        let face_cmd = self.face_to_angle(robot_state, target_angle, kp_theta, ki_theta, kd_theta);
        
        MotionCommand {
            id: robot_state.id,
            team: robot_state.team,
            vx: motion_cmd.vx,
            vy: motion_cmd.vy,
            omega: face_cmd.omega,
            orientation: robot_state.orientation,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_normalize_angle() {
        let pi = std::f64::consts::PI;
        assert!((Motion::normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((Motion::normalize_angle(pi) - pi).abs() < 1e-10);
        // -pi normalizado sigue siendo -pi (o muy cerca)
        let normalized_neg_pi = Motion::normalize_angle(-pi);
        assert!((normalized_neg_pi - (-pi)).abs() < 1e-10 || (normalized_neg_pi - pi).abs() < 1e-10);
        assert!((Motion::normalize_angle(2.0 * pi) - 0.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_move_direct() {
        let motion = Motion::new();
        let robot = RobotState::new(0, 0);
        let cmd = motion.move_direct(&robot, Vec2::new(1.0, 0.0));
        
        assert_eq!(cmd.id, 0);
        assert!(cmd.vx > 0.0);
        assert_eq!(cmd.vy, 0.0);
    }
}
