use glam::Vec2;
use crate::world::RobotState;

/// Controlador Bang-Bang para generar perfiles de velocidad suaves
/// con límites de aceleración y velocidad máxima
pub struct BangBangController {
    /// Aceleración máxima (m/s²)
    max_acceleration: f64,
    /// Velocidad máxima (m/s)
    max_velocity: f64,
    /// Velocidad actual (m/s) - se actualiza cada frame
    current_velocity: Vec2,
}

// Tipo auxiliar para cálculos en f64
struct Vec2F64 {
    x: f64,
    y: f64,
}

impl Vec2F64 {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    
    fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    
    fn normalize(&self) -> Self {
        let len = self.length();
        if len > 0.001 {
            Self {
                x: self.x / len,
                y: self.y / len,
            }
        } else {
            Self { x: 0.0, y: 0.0 }
        }
    }
    
    fn scale(&self, factor: f64) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
        }
    }
    
    fn add(&self, other: &Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
    
    fn sub(&self, other: &Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
    
    fn to_vec2(&self) -> Vec2 {
        Vec2::new(self.x as f32, self.y as f32)
    }
}

impl BangBangController {
    /// Crea un nuevo controlador Bang-Bang
    /// 
    /// # Argumentos
    /// * `max_acceleration` - Aceleración máxima en m/s² (ej: 2.5)
    /// * `max_velocity` - Velocidad máxima en m/s (ej: 5.0)
    pub fn new(max_acceleration: f64, max_velocity: f64) -> Self {
        Self {
            max_acceleration,
            max_velocity,
            current_velocity: Vec2::ZERO,
        }
    }

    /// Calcula el perfil de velocidad basado en la posición actual y objetivo
    pub fn compute_profile(
        &mut self,
        current_pos: Vec2,
        target_pos: Vec2,
        current_vel: Vec2,
        dt: f64,
    ) -> Vec2 {
        let result = self.compute_profile_with_velocity(current_pos, target_pos, current_vel, dt);
        self.current_velocity = result;
        result
    }

    /// Calcula el perfil de velocidad sin modificar el estado interno
    /// Útil cuando se quiere calcular para múltiples robots sin compartir estado
    pub fn compute_profile_with_velocity(
        &self,
        current_pos: Vec2,
        target_pos: Vec2,
        current_vel: Vec2,
        dt: f64,
    ) -> Vec2 {
        // Convertir a f64 para cálculos
        let current_pos_f64 = Vec2F64::new(current_pos.x as f64, current_pos.y as f64);
        let target_pos_f64 = Vec2F64::new(target_pos.x as f64, target_pos.y as f64);
        let current_vel_f64 = Vec2F64::new(current_vel.x as f64, current_vel.y as f64);
        
        // Vector hacia el objetivo
        let direction = target_pos_f64.sub(&current_pos_f64);
        let distance = direction.length();

        // Si está muy cerca del objetivo, detener
        if distance < 0.05 {
            return Vec2::ZERO;
        }

        // Dirección normalizada hacia el objetivo
        let direction_normalized = direction.normalize();

        // Velocidad deseada en la dirección del objetivo
        let current_speed = current_vel_f64.length();
        let desired_speed = self.compute_desired_speed(distance, current_speed, dt);
        let desired_velocity = direction_normalized.scale(desired_speed);
        
        // Aplicar límites de aceleración
        let velocity_change = desired_velocity.sub(&current_vel_f64);
        let velocity_change_magnitude = velocity_change.length();
        let max_velocity_change = self.max_acceleration * dt;
        
        let limited_velocity_change = if velocity_change_magnitude > max_velocity_change {
            let scale = max_velocity_change / velocity_change_magnitude;
            velocity_change.scale(scale)
        } else {
            velocity_change
        };
        
        // Nueva velocidad con límites de aceleración aplicados
        let new_velocity = current_vel_f64.add(&limited_velocity_change);
        let speed = new_velocity.length();
        
        // Limitar velocidad máxima
        let final_velocity = if speed > self.max_velocity {
            let scale = self.max_velocity / speed;
            new_velocity.scale(scale)
        } else {
            new_velocity
        };
        
        // Retornar como Vec2 (glam)
        final_velocity.to_vec2()
    }

    /// Calcula la velocidad deseada basada en la distancia y el perfil trapezoidal
    fn compute_desired_speed(&self, distance: f64, current_speed: f64, dt: f64) -> f64 {
        // Distancia necesaria para detenerse desde la velocidad actual
        let stop_distance = (current_speed * current_speed) / (2.0 * self.max_acceleration);
        
        // Si estamos muy cerca, detener
        if distance < 0.05 {
            return 0.0;
        }
        
        // Si estamos dentro de la distancia de parada, desacelerar
        if distance <= stop_distance {
            let decel_speed = (2.0 * self.max_acceleration * distance).sqrt();
            return decel_speed.min(current_speed);
        }
        
        // Para distancias cortas, usar velocidad proporcional a la distancia (más agresivo)
        if distance < 0.5 {
            return (distance * 4.0).min(self.max_velocity);
        }
        
        // Distancia necesaria para alcanzar velocidad máxima desde cero
        let acceleration_distance = (self.max_velocity * self.max_velocity) / (2.0 * self.max_acceleration);
        
        // Distancia necesaria para desacelerar desde velocidad máxima a cero
        let deceleration_distance = acceleration_distance;
        
        // Distancia total necesaria para el perfil completo
        let total_profile_distance = acceleration_distance + deceleration_distance;
        
        if distance >= total_profile_distance {
            // Fase de velocidad constante: usar velocidad máxima
            self.max_velocity
        } else if current_speed < self.max_velocity {
            // Fase de aceleración: acelerar hacia velocidad máxima (más agresivo)
            let new_speed = current_speed + self.max_acceleration * dt;
            new_speed.min(self.max_velocity)
        } else {
            // Ya estamos en velocidad máxima, mantenerla hasta que debamos desacelerar
            self.max_velocity
        }
    }

    /// Resetea la velocidad actual a cero
    pub fn reset(&mut self) {
        self.current_velocity = Vec2::ZERO;
    }

    /// Obtiene la velocidad actual
    pub fn get_current_velocity(&self) -> Vec2 {
        self.current_velocity
    }

    /// Obtiene la aceleración máxima
    pub fn get_max_acceleration(&self) -> f64 {
        self.max_acceleration
    }

    /// Obtiene la velocidad máxima
    pub fn get_max_velocity(&self) -> f64 {
        self.max_velocity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bang_bang_new() {
        let controller = BangBangController::new(2.5, 5.0);
        assert_eq!(controller.get_max_acceleration(), 2.5);
        assert_eq!(controller.get_max_velocity(), 5.0);
        assert_eq!(controller.get_current_velocity(), Vec2::ZERO);
    }

    #[test]
    fn test_bang_bang_reset() {
        let mut controller = BangBangController::new(2.5, 5.0);
        let _ = controller.compute_profile(
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::ZERO,
            0.016,
        );
        assert_ne!(controller.get_current_velocity(), Vec2::ZERO);
        
        controller.reset();
        assert_eq!(controller.get_current_velocity(), Vec2::ZERO);
    }

    #[test]
    fn test_bang_bang_stop_at_target() {
        let mut controller = BangBangController::new(2.5, 5.0);
        let vel = controller.compute_profile(
            Vec2::new(0.0, 0.0),
            Vec2::new(0.01, 0.0),
            Vec2::ZERO,
            0.016,
        );
        assert!(vel.length() < 0.1);
    }

    #[test]
    fn test_bang_bang_respects_max_velocity() {
        let mut controller = BangBangController::new(10.0, 2.0);
        let dt = 0.016;
        
        let mut current_pos = Vec2::new(0.0, 0.0);
        let target_pos = Vec2::new(10.0, 0.0);
        let mut current_vel = Vec2::ZERO;
        
        for _ in 0..100 {
            current_vel = controller.compute_profile(current_pos, target_pos, current_vel, dt);
            assert!(current_vel.length() <= 2.0 + 0.01);
            current_pos += current_vel * dt as f32;
        }
    }

    #[test]
    fn test_bang_bang_respects_max_acceleration() {
        let mut controller = BangBangController::new(2.5, 10.0);
        let dt = 0.016;
        
        let current_vel = Vec2::ZERO;
        let target_pos = Vec2::new(10.0, 0.0);
        let current_pos = Vec2::new(0.0, 0.0);
        
        let new_vel = controller.compute_profile(current_pos, target_pos, current_vel, dt);
        
        let velocity_change = new_vel.length() - current_vel.length();
        let max_change = controller.get_max_acceleration() * dt;
        
        assert!(velocity_change <= max_change + 0.001);
    }
}
