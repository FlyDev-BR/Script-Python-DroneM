from pymavlink import mavutil
import time

# Configura a conexão com o drone
connection_string = "udp:192.168.15.6:14550"
#connection_string = "udp:192.168.216.162:14550"  # Altere para o endereço correto (ex.: serial, udp, tcp)
master = mavutil.mavlink_connection(connection_string)

# Aguarda o heartbeat (sinal do drone)
print("Esperando pelo heartbeat...")
master.wait_heartbeat()
print("Conexão estabelecida com o drone!")

# Função para trocar o modo de voo
def set_mode(mode):
    # Obtém o ID do modo desejado
    mode_id = master.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Modo {mode} não disponível.")
        return

    # Envia o comando para alterar o modo
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    # Confirma a alteração
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(f"Modo alterado para {mode}: {ack_msg.result}")

# Função para armar os motores
def arm_motors():
    print("Armando motores...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 para armar, 0 para desarmar
        0, 0, 0, 0, 0, 0,
    )

    # Aguarda o comando ser aceito
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(f"Comando de armamento: {ack_msg.result}")

# Função para decolar
def takeoff(altitude):
    print(f"Iniciando decolagem para {altitude} metros...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude,
    )

    # Aguarda o comando ser aceito
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
    print(f"Comando de decolagem: {ack_msg.result}")

# Passo a passo para controlar o drone
set_mode("GUIDED")  # Configura o modo GUIDED
arm_motors()        # Arma os motores
time.sleep(5)       # Aguarda 2 segundos
takeoff(1)          # Decola e mantém 1 metro de altura
time.sleep(25)
set_mode("LAND")
