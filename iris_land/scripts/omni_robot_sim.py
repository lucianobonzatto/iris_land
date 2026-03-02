#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import pygame
import sys


class OmniRobotSimulator(Node):
    def __init__(self):
        super().__init__('omni_robot_simulator')
        
        # Subscriber para receber comandos de velocidade
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )
        
        # Publisher para publicar a posição atual
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )
        
        # Variáveis de estado do robô
        self.pos_x = 400.0  # Posição inicial X (centro da janela)
        self.pos_y = 300.0  # Posição inicial Y (centro da janela)
        self.vel_x = 0.0
        self.vel_y = 0.0
        
        # Configuração do pygame
        pygame.init()
        self.width = 800
        self.height = 600
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Robô Omnidirecional - Simulador')
        
        # Cores
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GRAY = (200, 200, 200)
        
        # Parâmetros do robô
        self.robot_radius = 20
        self.scale = 50.0  # Escala de pixels por metro
        
        # Timer para atualizar a simulação
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.update_simulation)
        
        self.get_logger().info('Simulador de robô omnidirecional iniciado')
        self.get_logger().info(f'Ouvindo comandos de velocidade no tópico: /cmd_vel')
        self.get_logger().info(f'Publicando posição no tópico: /robot_pose')
        
    def velocity_callback(self, msg):
        """Callback para atualizar as velocidades do robô"""
        self.vel_x = msg.linear.x
        self.vel_y = -msg.linear.y
        self.get_logger().debug(f'Velocidade recebida - X: {self.vel_x:.2f}, Y: {self.vel_y:.2f}')
    
    def publish_pose(self):
        """Publica a posição atual do robô em metros"""
        pose_msg = PoseStamped()
        
        # Header com timestamp
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        # Converter posição de pixels para metros (centralizado na origem)
        pose_msg.pose.position.x = -(self.pos_y - self.height / 2) / self.scale
        pose_msg.pose.position.y = (self.pos_x - self.width / 2) / self.scale
        pose_msg.pose.position.z = 0.0
        
        # Orientação (quaternion identidade - sem rotação)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_publisher.publish(pose_msg)
        self.get_logger().debug(f'Posição: X={pose_msg.pose.position.x:.2f}m, Y={pose_msg.pose.position.y:.2f}m')
        
    def update_simulation(self):
        """Atualiza a posição do robô e redesenha a tela"""
        # Processar eventos do pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info('Fechando simulador...')
                rclpy.shutdown()
                pygame.quit()
                sys.exit()
                
        # Atualizar posição baseado na velocidade
        # No ROS, Y positivo é para frente, X positivo é para esquerda
        # No pygame, Y positivo é para baixo, X positivo é para direita
        # Ajustamos a direção do Y e invertemos X e Y para corresponder ao movimento do robô
        self.pos_x += self.vel_y * self.scale * self.dt  # vel_y do ROS move em X na tela
        self.pos_y -= self.vel_x * self.scale * self.dt  # vel_x do ROS move em Y na tela (invertido)
        
        # Limitar posição dentro da janela
        self.pos_x = max(self.robot_radius, min(self.width - self.robot_radius, self.pos_x))
        self.pos_y = max(self.robot_radius, min(self.height - self.robot_radius, self.pos_y))
        
        # Publicar posição atual (converter pixels para metros)
        self.publish_pose()
        
        # Desenhar
        self.draw()
        
    def draw(self):
        """Desenha o robô na tela"""
        # Limpar tela com fundo branco
        self.screen.fill(self.WHITE)
        
        # Desenhar grade de referência
        grid_spacing = 50
        for x in range(0, self.width, grid_spacing):
            pygame.draw.line(self.screen, self.GRAY, (x, 0), (x, self.height), 1)
        for y in range(0, self.height, grid_spacing):
            pygame.draw.line(self.screen, self.GRAY, (0, y), (self.width, y), 1)
            
        # Desenhar o robô (círculo preto)
        pygame.draw.circle(self.screen, self.BLACK, 
                         (int(self.pos_x), int(self.pos_y)), 
                         self.robot_radius)
        
        # Desenhar indicador de direção (linha pequena mostrando orientação)
        if abs(self.vel_x) > 0.01 or abs(self.vel_y) > 0.01:
            end_x = int(self.pos_x + self.vel_y * 15)
            end_y = int(self.pos_y - self.vel_x * 15)
            pygame.draw.line(self.screen, (255, 0, 0), 
                           (int(self.pos_x), int(self.pos_y)), 
                           (end_x, end_y), 3)
        
        # Atualizar display
        pygame.display.flip()


def main(args=None):
    rclpy.init(args=args)
    node = OmniRobotSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
