from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
import math


def armar(logan):
    logan.mode = VehicleMode("GUIDED")
    logan.armed = True

    if logan.mode != VehicleMode("GUIDED"):
        time.sleep(1)
    
#------------------------------#

def decolar(logan, altitude):
    print(F"Decolando até {altitude}m...")
    logan.simple_takeoff(altitude)

    while True:
        print(F"Altitude = {logan.location.global_relative_frame.alt}m")

        while logan.location.global_relative_frame.alt < altitude:
            posicao_drone(logan)
            if logan.location.global_relative_frame.alt >= altitude*0.95:
                print("Atingiu a altitude declarada.")
                break
            time.sleep(1)
            
        posicao_drone(logan)

        time.sleep(1)
        break
    
#------------------------------#

def liberar_gancho():
        pwm_open = 1900
        pwm_close = 1100

        print("Liberando gancho...")
        servo(7, pwm_open)
        servo(8, pwm_open)
        time.sleep(1)
    
#------------------------------#

def mudar_param(logan, nome_param, valor): #se ligar na unidade de medida do valor do param!!!!!!!!
    print(f"Mudando o parametro {nome_param} para {valor}")
    logan.parameters[nome_param] = valor
    time.sleep(1)
    leitura_de_valor = logan.parameters[nome_param]
    if math.isclose(leitura_de_valor, valor):
        print(f"Parametro {nome_param} mudado com SUCESSO para {leitura_de_valor}")
        return True
    
    else:
        print(f"Parametro {nome_param} FALHOU na mudanca de valor.")
        return False
    
#------------------------------#

def posicao_drone(logan):
    altitude_atual = logan.location.global_relative_frame.alt
    dist_norte = logan.location.local_frame.north
    dist_leste = logan.location.local_frame.east
    print(f"Altitude(em relação ao solo: {altitude_atual}m)")
    print(f"Posição em metros em relação ao NORTE: {dist_norte:.2f}m")
    print(f"Posição em metros em relação ao LESTE: {dist_leste:.2f}m")
   
#------------------------------#

def pousar(logan):
    logan.mode = VehicleMode("LAND")
    time.sleep(1)
    if logan.mode.name != "LAND":
        logan.mode = VehicleMode("LAND")
        while not logan.mode.name == "LAND":
            time.sleep(1)
        print("Drone esta no modo LAND")

    else:
        while logan.location.global_relative_frame.alt <= 0.02:
            posicao_drone()
            time.sleep(1)
        print("Veículo pousado.")
        logan.armed = False
    
#------------------------------#

def rtl(logan):
    logan.mode = VehicleMode("RTL")
    
    if logan.mode.name != "RTL":
        logan.mode = VehicleMode("RTL")
    
    while not logan.mode.name == "RTL":
        time.sleep(1)

    print("Drone está no modo RTL.")
    while logan.mode.name == "RTL":
        posicao_drone(logan)
        time.sleep(1)
    
    if logan.location.global_relative_frame.alt <= 0.02:
        logan.mode = VehicleMode("LAND")
        logan.armed = False
    
#------------------------------#   

def servo(logan, channel, pwm):
    logan._master.mav.command_long_send(
    logan._master.target_system,
    logan._master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    channel,    # Canal servo (7 ou 8)
    pwm,        # PWM value (ex: 1000-2000)
    0, 0, 0, 0, 0
)
    
#------------------------------#

def liberar_gancho():
        pwm_open = 1900
        pwm_close = 1100

        print("Liberando gancho...")
        servo(7, pwm_open)
        servo(8, pwm_open)
        time.sleep(1)
    
#------------------------------#    

def status_voo(logan):

    print("STATUS DE VOO DO LOGAN:")

    print("\nConexao e Sistema: ")

    print(f"Modo de Voo: {logan.mode.name}")
    print(f"Status do Sistema: {logan.system_status.state}")
    print(f"Heartbeat do drone: {logan.last.heartbeat}")
    print(f"Status do sistema do drone: {logan.system_status.state}")

    #-------------------------------------------#

    print("\nOrientaçoes:")
    pitch_graus = math.degrees(logan.attitude.pitch)
    roll_graus = math.degrees(logan.attitude.roll)
    yaw_graus = math.degrees(logan.attitude.yaw)

    print(f"Pitch (Arfagem): {pitch_graus:.2f}°")
    print(f"Roll (Rolagem): {roll_graus:.2f}°")
    print(f"Yaw (Guinada/Proa): {yaw_graus:.2f}°")

    #-------------------------------------------#

    print("\nVelocidade: ")
    print(f"Velocidade em relação ao solo (Groundspeed): {logan.groundspeed:.2f} m/s")
    print(f"Velocidade em relação ao ar (Airspeed): {logan.airspeed:.2f} m/s")
    print(f"Velocidade Vertical (Vz): {logan.velocity[2]:.2f} m/s")

    #-------------------------------------------#
  
    print("\nBateria:")
    print(f"Voltagem: {logan.battery.voltage:.2f}V")
    print(f"Corrente: {logan.battery.current}A") 
    print(f"Nível: {logan.battery.level}%")

    '''
    -> MINIMO DE VOLTAGEM PARA COMPETICAO!! retornar isso no script das missoes.
    
    if logan.battery.voltage <= 15.95:
        print(f"Bateria abaixo do determinado pela SAE..")
        print("\nAcionando LAND...")
        logan.mode = VehicleMode("LAND")

        while logan.mode.name != "LAND":
            time. sleep(0.5)
        print("MOdo LAND ativado. Drone pousando...")
        if logan.location.global_relative_frame.alt <= 0.02:
            logan.armed = False
            print("Drone desarmado.")
    '''


def velocidade(logan,veloc_x,veloc_y,veloc_z,duracao): 

    msg = logan.message_factory.set_position_target_local_ned_encode(
        0,  #time_boot_ms(nao usado)
        0, 0, #target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, # type_mask (only speeds enabled)
        0,0,0,  # x, y, z positions (nao usado)
        veloc_x, veloc_y, veloc_z, #m/s
        0,0,0, # x,y, z acceleration (n suportada no GCS mavlink**)
        0,
        0)

    #for x in range(0,duracao):
       # logan.send_mavlink(msg)
       # time.sleep(1)
    end_time = time.time() + duracao
    while time.time() < end_time:
        logan.send_mavlink(msg)
        time.sleep(0.1)
            
#------------------------------#

def yaw(logan, angulo, relative = True):
    if relative:
        is_relative = 1
    else:
        is_relative = 0
    


    msg = logan.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        angulo,
        0,
        1,
        is_relative,
        0,0,0
    )
    logan.send_mavlink(msg) 

def manter_posicao(logan, duracao):
    velocidade(logan, 0, 0, 0, duracao) 

def mover_frente_tras(logan, veloc_x, duracao): #frente = +x, tras = -x
    velocidade(logan,veloc_x, 0, 0, duracao)

def mover_lateral(logan, veloc_y, duracao): #direita = +y, esquera = -y 
    velocidade(logan, 0, veloc_y, 0, duracao)

def mover_subir_descer(logan, veloc_z,duracao): # down = +z, up = -z
    velocidade(logan, 0, 0, veloc_z, duracao)

def girar_drone(logan, angulo, relative = True):
    if angulo > 0:
        print(f"Girando para direita {angulo} graus...")
        yaw(logan, angulo, relative=True)

    else:
        #calculo para saber o novo angulo 0 - 360:
        ang_atual = math.degrees(logan.attitude.yaw)
        if ang_atual == 0:
            ang_novo = 360 - angulo
        else:
            ang_novo = ang_atual - angulo
        
        angulo = ang_novo
        yaw(logan, angulo, relative = True)
        print(f"Girando para esquerda {angulo} graus...")
