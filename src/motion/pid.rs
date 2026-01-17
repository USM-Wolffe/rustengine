/// Controlador PID para control de movimiento
pub struct PIDController {
    kp: f64,          // Ganancia proporcional
    ki: f64,          // Ganancia integral
    kd: f64,          // Ganancia derivativa
    integral: f64,    // Acumulador integral
    last_error: f64,  // Error anterior (para derivativa)
}

impl PIDController {
    /// Crea un nuevo controlador PID con las ganancias especificadas
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            last_error: 0.0,
        }
    }
    
    /// Calcula la salida del controlador PID
    pub fn compute(&mut self, error: f64, dt: f64) -> f64 {
        // Término proporcional
        let p_term = self.kp * error;
        
        // Término integral
        self.integral += error * dt;
        let i_term = self.ki * self.integral;
        
        // Término derivativo
        let d_term = if dt > 0.0 {
            self.kd * (error - self.last_error) / dt
        } else {
            0.0
        };
        
        self.last_error = error;
        
        p_term + i_term + d_term
    }
    
    /// Resetea el acumulador integral
    pub fn reset_integral(&mut self) {
        self.integral = 0.0;
    }
    
    /// Resetea completamente el controlador
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
    }
    
    /// Obtiene la ganancia proporcional
    pub fn get_kp(&self) -> f64 {
        self.kp
    }
    
    /// Obtiene la ganancia integral
    pub fn get_ki(&self) -> f64 {
        self.ki
    }
    
    /// Obtiene la ganancia derivativa
    pub fn get_kd(&self) -> f64 {
        self.kd
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pid_controller() {
        let mut pid = PIDController::new(1.0, 0.1, 0.01);
        
        // Test con error constante
        let output1 = pid.compute(1.0, 0.1);
        assert!(output1 > 0.0);
        
        // El término integral debe acumularse (output2 debería ser mayor)
        let output2 = pid.compute(1.0, 0.1);
        // Con ki=0.1, el término integral aumenta: output2 = output1 + 0.1*1.0*0.1 = output1 + 0.01
        // output1 = 1.0*1.0 + 0.1*0.1 + 0.01*0 = 1.0 + 0.01 = 1.01
        // output2 = 1.0*1.0 + 0.1*0.2 + 0.01*0 = 1.0 + 0.02 = 1.02
        assert!(output2 > output1 - 0.001); // Permitir pequeña diferencia numérica
    }
    
    #[test]
    fn test_pid_reset() {
        let mut pid = PIDController::new(1.0, 0.1, 0.01);
        pid.compute(1.0, 0.1);
        pid.reset();
        
        // Después del reset, el output debe ser igual al primero
        let output1 = pid.compute(1.0, 0.1);
        let mut pid2 = PIDController::new(1.0, 0.1, 0.01);
        let output2 = pid2.compute(1.0, 0.1);
        assert!((output1 - output2).abs() < 1e-10);
    }
}
