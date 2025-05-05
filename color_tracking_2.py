from djitellopy import Tello 
import cv2 
import time 

drone = Tello()
drone.connect()

drone.streamon()
time.sleep(3)

global flying

def clean_exit(): 
    print("\nCerrando el prgrama...")

    if flying: 
        drone.send_rc_control(0, 0, 0, 0)
        time.sleep(0.5)
        drone.land()
    cv2.destroyAllWindows()
    drone.streamoff()
    drone.end()
    print("Programa cerrado correctamente")

def control(): 
    global flying
    flying = False 
    fb_vel = 0

    while True:
        frame = drone.get_frame_read().frame
        if frame is None:
            print("Frame no recibido")
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        print(f"Tamaño del frame: {frame.shape}")

        cv2.putText(frame, f'Bateria: {drone.get_battery()}%',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)

        cv2.putText(frame, f'Altura: {drone.get_height()}cm',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)
        if drone.get_height() > 0: Marco = "Volando"
        else: Marco = "Detenido"
        cv2.putText(frame, f'Estado: {Marco}cm',
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)
        
        cv2.imshow('Video Stream', frame)

        key = cv2.waitKey(1) & 0xFF

def main():
    try:
        control()
    except KeyboardInterrupt:
        clean_exit()

if __name__ == '__main__':
    main()

