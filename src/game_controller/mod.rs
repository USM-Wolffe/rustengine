use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use std::net::{Ipv4Addr, SocketAddr};
use std::error::Error;
use std::sync::Arc;
use tokio::sync::RwLock as TokioRwLock;
use crate::protos::ssl_gc_referee_message::{Referee, referee};
use protobuf::Message;

// --- Estado del Juego ---

/// Estado completo del juego según el referee
#[derive(Debug, Clone)]
pub struct GameState {
    /// Etapa actual del juego (primera mitad, segunda mitad, etc.)
    pub stage: referee::Stage,
    /// Tiempo restante en la etapa (microsegundos)
    pub stage_time_left: Option<i64>,
    /// Comando actual del referee (HALT, STOP, RUNNING, etc.)
    pub command: referee::Command,
    /// Contador de comandos (mod 2^32)
    pub command_counter: u32,
    /// Timestamp del comando (microsegundos)
    pub command_timestamp: u64,
    /// Información del equipo amarillo
    pub yellow_team: TeamInfo,
    /// Información del equipo azul
    pub blue_team: TeamInfo,
    /// Posición designada para ball placement (si aplica)
    pub designated_position: Option<glam::Vec2>,
    /// Si el equipo azul juega en el lado positivo del eje X
    pub blue_team_on_positive_half: bool,
    /// Próximo comando después del ball placement
    pub next_command: Option<referee::Command>,
    /// Tiempo restante para la acción actual (microsegundos)
    pub current_action_time_remaining: Option<i64>,
    /// Mensaje de estado para espectadores
    pub status_message: Option<String>,
}

/// Información de un equipo
#[derive(Debug, Clone)]
pub struct TeamInfo {
    /// Nombre del equipo
    pub name: String,
    /// Goles anotados
    pub score: u32,
    /// Tarjetas rojas
    pub red_cards: u32,
    /// Tiempo restante en cada tarjeta amarilla (microsegundos)
    pub yellow_card_times: Vec<u32>,
    /// Total de tarjetas amarillas
    pub yellow_cards: u32,
    /// Timeouts restantes
    pub timeouts: u32,
    /// Tiempo de timeout disponible (microsegundos)
    pub timeout_time: u32,
    /// ID del portero
    pub goalkeeper: u32,
}

impl Default for GameState {
    fn default() -> Self {
        Self {
            stage: referee::Stage::NORMAL_FIRST_HALF_PRE,
            stage_time_left: None,
            command: referee::Command::HALT,
            command_counter: 0,
            command_timestamp: 0,
            yellow_team: TeamInfo::default(),
            blue_team: TeamInfo::default(),
            designated_position: None,
            blue_team_on_positive_half: false,
            next_command: None,
            current_action_time_remaining: None,
            status_message: None,
        }
    }
}

impl Default for TeamInfo {
    fn default() -> Self {
        Self {
            name: String::new(),
            score: 0,
            red_cards: 0,
            yellow_card_times: Vec::new(),
            yellow_cards: 0,
            timeouts: 0,
            timeout_time: 0,
            goalkeeper: 0,
        }
    }
}

// --- Eventos del GameController ---

#[derive(Debug, Clone)]
pub enum GameControllerEvent {
    /// Nuevo mensaje del referee recibido
    RefereeMessage(GameState),
    /// Cambio de comando
    CommandChanged(referee::Command),
    /// Cambio de etapa
    StageChanged(referee::Stage),
}

// --- Módulo GameController ---

pub struct GameController {
    multicast_ip: String,
    port: u16,
    socket: Option<UdpSocket>,
}

impl GameController {
    /// Crea un nuevo GameController
    pub fn new(multicast_ip: String, port: u16) -> Self {
        Self {
            multicast_ip,
            port,
            socket: None,
        }
    }

    /// Inicializa el socket UDP y se une al grupo multicast
    pub async fn initialize(&mut self) -> Result<(), Box<dyn Error + Send + Sync>> {
        let addr = format!("0.0.0.0:{}", self.port);
        let socket = UdpSocket::bind(&addr).await?;
        
        // Unirse al grupo multicast (GameController usa multicast en 224.5.23.1)
        let multicast_addr: Ipv4Addr = self.multicast_ip.parse()?;
        socket.join_multicast_v4(multicast_addr, Ipv4Addr::UNSPECIFIED)?;
        
        eprintln!("[GameController] Escuchando en {}:{}", self.multicast_ip, self.port);
        self.socket = Some(socket);
        Ok(())
    }

    /// Recibe un mensaje del referee (no bloquea)
    pub async fn receive_message(&self) -> Result<Option<Referee>, Box<dyn Error + Send + Sync>> {
        let socket = match &self.socket {
            Some(s) => s,
            None => return Err("Socket no inicializado".into()),
        };

        let mut buf = vec![0u8; 65536]; // Buffer grande para mensajes grandes
        match socket.recv_from(&mut buf).await {
            Ok((size, _addr)) => {
                let data = &buf[..size];
                
                // Parsear mensaje protobuf
                match Referee::parse_from_bytes(data) {
                    Ok(referee_msg) => {
                        Ok(Some(referee_msg))
                    }
                    Err(e) => {
                        eprintln!("[GameController] Error parseando mensaje: {}", e);
                        Ok(None)
                    }
                }
            }
            Err(e) => {
                // Timeout o error, no es crítico
                if e.kind() == std::io::ErrorKind::TimedOut || 
                   e.kind() == std::io::ErrorKind::WouldBlock {
                    Ok(None)
                } else {
                    Err(format!("Error recibiendo mensaje: {}", e).into())
                }
            }
        }
    }

    /// Convierte un mensaje Referee protobuf a GameState
    pub fn parse_referee_message(referee_msg: &Referee) -> GameState {
        let yellow_info = if let Some(ref info) = referee_msg.yellow.as_ref() {
            TeamInfo {
                name: info.name().to_string(),
                score: info.score(),
                red_cards: info.red_cards(),
                yellow_card_times: info.yellow_card_times.clone(),
                yellow_cards: info.yellow_cards(),
                timeouts: info.timeouts(),
                timeout_time: info.timeout_time(),
                goalkeeper: info.goalkeeper(),
            }
        } else {
            TeamInfo::default()
        };
        
        let blue_info = if let Some(ref info) = referee_msg.blue.as_ref() {
            TeamInfo {
                name: info.name().to_string(),
                score: info.score(),
                red_cards: info.red_cards(),
                yellow_card_times: info.yellow_card_times.clone(),
                yellow_cards: info.yellow_cards(),
                timeouts: info.timeouts(),
                timeout_time: info.timeout_time(),
                goalkeeper: info.goalkeeper(),
            }
        } else {
            TeamInfo::default()
        };
        
        let designated_pos = referee_msg.designated_position.as_ref().map(|dp| {
            glam::Vec2::new(dp.x(), dp.y()) / 1000.0 // Convertir mm a m
        });

        GameState {
            stage: referee_msg.stage(),
            stage_time_left: referee_msg.stage_time_left,
            command: referee_msg.command(),
            command_counter: referee_msg.command_counter(),
            command_timestamp: referee_msg.command_timestamp(),
            yellow_team: yellow_info,
            blue_team: blue_info,
            designated_position: designated_pos,
            blue_team_on_positive_half: referee_msg.blue_team_on_positive_half(),
            next_command: if referee_msg.has_next_command() {
                Some(referee_msg.next_command())
            } else {
                None
            },
            current_action_time_remaining: referee_msg.current_action_time_remaining,
            status_message: referee_msg.status_message.clone(),
        }
    }

    /// Ejecuta el GameController en un loop continuo
    pub async fn run(
        &mut self,
        state_tx: Arc<TokioRwLock<GameState>>,
        event_tx: Option<mpsc::Sender<GameControllerEvent>>,
    ) -> Result<(), Box<dyn Error + Send + Sync>> {
        // Inicializar socket
        self.initialize().await?;

        let mut last_command = referee::Command::HALT;
        let mut last_stage = referee::Stage::NORMAL_FIRST_HALF_PRE;

        loop {
            match self.receive_message().await {
                Ok(Some(referee_msg)) => {
                    let game_state = Self::parse_referee_message(&referee_msg);
                    
                    // Actualizar estado compartido
                    {
                        let mut state = state_tx.write().await;
                        *state = game_state.clone();
                    }

                    // Enviar eventos si hay cambios
                    if let Some(ref tx) = event_tx {
                        // Evento de nuevo mensaje
                        let _ = tx.send(GameControllerEvent::RefereeMessage(game_state.clone())).await;

                        // Evento de cambio de comando
                        if game_state.command != last_command {
                            let _ = tx.send(GameControllerEvent::CommandChanged(game_state.command)).await;
                            last_command = game_state.command;
                            eprintln!("[GameController] Comando cambiado: {:?}", game_state.command);
                        }

                        // Evento de cambio de etapa
                        if game_state.stage != last_stage {
                            let _ = tx.send(GameControllerEvent::StageChanged(game_state.stage)).await;
                            last_stage = game_state.stage;
                            eprintln!("[GameController] Etapa cambiada: {:?}", game_state.stage);
                        }
                    }

                    // Log periódico del estado
                    if game_state.command_counter % 100 == 0 {
                        eprintln!(
                            "[GameController] Stage: {:?}, Command: {:?}, Yellow: {} ({}), Blue: {} ({})",
                            game_state.stage,
                            game_state.command,
                            game_state.yellow_team.name,
                            game_state.yellow_team.score,
                            game_state.blue_team.name,
                            game_state.blue_team.score
                        );
                    }
                }
                Ok(None) => {
                    // Timeout, continuar esperando
                }
                Err(e) => {
                    eprintln!("[GameController] Error: {}", e);
                    // Continuar intentando recibir mensajes
                    tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                }
            }
        }
    }
}

// --- Funciones de utilidad ---

impl GameState {
    /// Verifica si el juego está en estado HALT
    pub fn is_halted(&self) -> bool {
        self.command == referee::Command::HALT
    }

    /// Verifica si el juego está detenido (HALT o STOP)
    pub fn is_stopped(&self) -> bool {
        matches!(self.command, referee::Command::HALT | referee::Command::STOP)
    }

    /// Verifica si el juego está corriendo (NORMAL_START o FORCE_START)
    pub fn is_running(&self) -> bool {
        matches!(
            self.command,
            referee::Command::NORMAL_START | referee::Command::FORCE_START
        )
    }

    /// Verifica si es un comando de ball placement
    pub fn is_ball_placement(&self) -> bool {
        matches!(
            self.command,
            referee::Command::BALL_PLACEMENT_YELLOW | referee::Command::BALL_PLACEMENT_BLUE
        )
    }

    /// Obtiene el equipo que tiene el ball placement
    pub fn ball_placement_team(&self) -> Option<i32> {
        match self.command {
            referee::Command::BALL_PLACEMENT_YELLOW => Some(1), // Yellow = 1
            referee::Command::BALL_PLACEMENT_BLUE => Some(0),   // Blue = 0
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_game_state_default() {
        let state = GameState::default();
        assert_eq!(state.command, referee::Command::HALT);
        assert_eq!(state.stage, referee::Stage::NORMAL_FIRST_HALF_PRE);
        assert_eq!(state.command_counter, 0);
    }

    #[test]
    fn test_game_state_helpers() {
        let mut state = GameState::default();
        
        state.command = referee::Command::HALT;
        assert!(state.is_halted());
        assert!(state.is_stopped());
        assert!(!state.is_running());

        state.command = referee::Command::STOP;
        assert!(!state.is_halted());
        assert!(state.is_stopped());
        assert!(!state.is_running());

        state.command = referee::Command::NORMAL_START;
        assert!(!state.is_halted());
        assert!(!state.is_stopped());
        assert!(state.is_running());

        state.command = referee::Command::BALL_PLACEMENT_BLUE;
        assert!(state.is_ball_placement());
        assert_eq!(state.ball_placement_team(), Some(0));

        state.command = referee::Command::BALL_PLACEMENT_YELLOW;
        assert!(state.is_ball_placement());
        assert_eq!(state.ball_placement_team(), Some(1));
    }
}