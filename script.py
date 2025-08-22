from pymavlink import mavutil
from gpiozero import LED
from time import sleep

# Configura o LED
led = LED(17)  # Use o pino correto para seu LED

# Estabelece conexão MAVLink com o computador
master = mavutil.mavlink_connection('udp://:14550')  # Ou a string correta de conexão

# Espera pelo heartbeat
master.wait_heartbeat()

print("Conexão MAVLink estabelecida.")

while True:
    msg = master.recv_match(type='COMMAND_LONG', blocking=True)  # Espera por comandos específicos
    if msg:
        print(f"Comando recebido: {msg}")
        
        # Verifique se o comando é o que você quer para acionar o LED
        if msg.command == 200:  # Comando fictício para acender o LED (escolha um número ou comando apropriado)
            if msg.param1 == 1:  # Parâmetro que define se o LED deve ser aceso ou apagado
                led.on()
                print("LED ligado")
            elif msg.param1 == 0:
                led.off()
                print("LED apagado")
