#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import radians
from std_msgs.msg import String

class TMRobot:
    def __init__(self):
        # 1. Inicializa o MoveIt e o Nó ROS
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('tm_robot_controller', anonymous=True)

        # 2. Configura o grupo de planejamento (O nome do grupo geralmente é 'manipulator' ou 'tmr_arm')
        # Verifique no seu setup se o nome é 'tmr_arm' ou 'manipulator'
        group_name = "tmr_arm" 
        try:
            self.move_group = moveit_commander.MoveGroupCommander(group_name)
        except RuntimeError:
            rospy.logerr("ERRO: Não foi possivel conectar ao grupo '%s'. Verifique se o MoveIt está rodando.", group_name)
            sys.exit(1)

        # Configurações de tolerância e velocidade
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.01)
        self.move_group.set_max_velocity_scaling_factor(0.3) # 30% da velocidade máxima
        self.move_group.set_max_acceleration_scaling_factor(0.3)

        # 3. Definição das Posições (JÁ CONVERTIDO PARA RADIANOS)
        # Ordem das juntas: Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3
        self.locais = {
            "direita": [
                radians(-75.38), radians(23), radians(110.88), 
                radians(-48.65), radians(90.53), radians(-0.21)
            ],
            "esquerda": [
                radians(94.995), radians(16.84), radians(118.95), 
                radians(-48.67), radians(90.53), radians(-0.21)
            ],
            "pegar": [ # "pegarpeça"
                radians(7.74), radians(60.04), radians(82.60), 
                radians(-48.67), radians(90.53), radians(-0.21)
            ],
            "home": [
                radians(7.75), radians(-30.28), radians(85.44), 
                radians(-48.67), radians(90.53), radians(-0.21)
            ]
        }

        # Cria o Subscriber (Ouvinte)
        # Tópico: /comando_robo
        # Tipo: String
        rospy.Subscriber("/comando_robo", String, self.callback_comando)
        
        rospy.loginfo("--------------------------------------------------")
        rospy.loginfo("ROBO TM PRONTO! Aguardando comandos no topico '/comando_robo'")
        rospy.loginfo("Comandos validos: 'home', 'esquerda', 'direita', 'pegar'")
        rospy.loginfo("--------------------------------------------------")

    def mover_para(self, nome_local):
        if nome_local not in self.locais:
            rospy.logwarn("Local '%s' nao existe no dicionario!", nome_local)
            return

        alvo = self.locais[nome_local]
        rospy.loginfo("Movendo para: %s ...", nome_local)
        
        # Envia o comando para o MoveIt
        try:
            self.move_group.go(alvo, wait=True)
            self.move_group.stop() # Garante que parou
            rospy.loginfo("Chegou em %s!", nome_local)
        except Exception as e:
            rospy.logerr("Erro ao mover: %s", str(e))

    def callback_comando(self, msg):
        """
        Função chamada toda vez que chega uma mensagem no tópico /comando_robo
        """
        comando = msg.data.lower().strip() # Transforma em minúsculo e tira espaços
        
        if comando == "pegarpeça" or comando == "pegar_peca":
            comando = "pegar"

        rospy.loginfo(">>> COMANDO RECEBIDO: %s", comando)
        
        if comando in self.locais:
            self.mover_para(comando)
        else:
            rospy.logwarn("Comando desconhecido: %s", comando)

if __name__ == '__main__':
    try:
        controlador = TMRobot()
        # Mantém o script rodando esperando mensagens
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass