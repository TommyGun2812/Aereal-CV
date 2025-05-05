from djitellopy import Tello 
import cv2 
import time 
import numpy as np 

#Image rescaling
width = 800
height = 600

# x-y axis thresholds
x_threshold = int(0.10 * width)
y_threshold = int(0.10 * height)

#Valores iniciales
H_min_init = 90
H_max_init = 150
S_min_init = 50 
S_max_init = 200 
V_min_init = 80
V_max_init = 170

#Minimum area for object detection
area_min = 0.05 * (width * height)


# Tello class instance
drone = Tello()
drone.connect()

drone.streamon()
time.sleep(3)

global flying

def callback(x): 
    pass

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

    # Crea una ventana para las trackbars
    cv2.namedWindow('Trackbars')
    # Define un tamaño personalizado para la ventana de trackbars
    cv2.resizeWindow('Trackbars', 600, 250)  # Ancho = 600 píxeles, Alto = 250 píxeles

    # Crea trackbars para ajustar los valores H, S, V
    cv2.createTrackbar('H Min', 'Trackbars', H_min_init, 179, callback)
    cv2.createTrackbar('H Max', 'Trackbars', H_max_init, 179, callback)
    cv2.createTrackbar('S Min', 'Trackbars', S_min_init, 255, callback)
    cv2.createTrackbar('S Max', 'Trackbars', S_max_init, 255, callback)
    cv2.createTrackbar('V Min', 'Trackbars', V_min_init, 255, callback)
    cv2.createTrackbar('V Max', 'Trackbars', V_max_init, 255, callback)

    while True:
        frame = drone.get_frame_read().frame
        if frame is None:
            continue
        
        frame = cv2.resize(frame, (width, height))
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Convierte la imagen de BGR a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Obtiene los valores de las trackbars
        h_min = cv2.getTrackbarPos('H Min', 'Trackbars')
        s_min = cv2.getTrackbarPos('S Min', 'Trackbars')
        v_min = cv2.getTrackbarPos('V Min', 'Trackbars')
        h_max = cv2.getTrackbarPos('H Max', 'Trackbars')
        s_max = cv2.getTrackbarPos('S Max', 'Trackbars')
        v_max = cv2.getTrackbarPos('V Max', 'Trackbars')

        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        # Aplica desenfoque gaussiano para reducir el ruido
        blurred = cv2.GaussianBlur(hsv, (15, 15), 0)
        mask = cv2.inRange(blurred, lower_hsv, upper_hsv)

        # Erosión y dilatación para eliminar ruido en la máscara
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Aplica la máscara a la imagen original
        result = cv2.bitwise_and(frame, frame, mask=mask)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_min: 
                cv2.drawContours(frame, contour, -1, (255, 0, 255), 7)
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 5)
                center = (x + w // 2, y + h // 2)
                cv2.circle(frame, center, 5, (0, 0, 255), cv2.FILLED)

                #Revisar si el objeto está a la derecha de la imagen
                if x + w//2 > width // 2 + x_threshold:
                    cv2.putText(frame, f'Objeto a la derecha', (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    
                elif x + w//2 < width // 2 - x_threshold:
                    cv2.putText(frame, f'Objeto a la izquierda', (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                else: 
                    cv2.putText(frame, f'Objeto en Rango', (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    
                #Revisar si el objeto está a la arriba de la imagen
                if y + h//2 > height // 2 + y_threshold:
                    cv2.putText(frame, f'Objeto arriba', (10, 100), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    
                elif y + h//2 < height // 2 - y_threshold:
                    cv2.putText(frame, f'Objeto abajo', (10, 100), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                else: 
                    cv2.putText(frame, f'Objeto en Rango', (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                
                    #Dibujar líneas de referencia 
            cv2.line(frame, (width//2 - x_threshold, 0), (width//2 - x_threshold, height), (255, 0, 0), 3)
            cv2.line(frame, (width//2 + x_threshold, 0), (width//2 + x_threshold, height), (255, 0, 0), 3)
            cv2.line(frame, (0, height//2 - y_threshold), (width, height//2 - y_threshold), (255, 0, 0), 3)
            cv2.line(frame, (0, height//2 + y_threshold), (width, height//2 + y_threshold), (255, 0, 0), 3)

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
        
        cv2.imshow('Original', frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Filtrado', result)

        key = cv2.waitKey(1) & 0xFF

def main():
    try:
        control()
    except KeyboardInterrupt:
        clean_exit()

if __name__ == '__main__':
    main()

