# Import libraries
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
H_min_init = 0
H_max_init = 0
S_min_init = 0 
S_max_init = 0
V_min_init = 0
V_max_init = 0

#Minimum area for object detection
area_min = 0.05 * (width * height)


# Tello class instance
drone = Tello()
drone.connect()

#Initializing drone stream
drone.streamon()
time.sleep(3)

#Global drone's state variabel
global flying

def callback(x): 
    pass

# Function to clean exit
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

# Conrol function
def control(): 
    global flying
    flying = False 
    fb_vel = 0
    prev = [0,0,0,0]


    # Trackbar window
    cv2.namedWindow('Trackbars')
    # Define trackbar widow size
    cv2.resizeWindow('Trackbars', 600, 250)  # Ancho = 600 píxeles, Alto = 250 píxeles

    # Creating trackbars
    cv2.createTrackbar('H Min', 'Trackbars', H_min_init, 179, callback)
    cv2.createTrackbar('H Max', 'Trackbars', H_max_init, 179, callback)
    cv2.createTrackbar('S Min', 'Trackbars', S_min_init, 255, callback)
    cv2.createTrackbar('S Max', 'Trackbars', S_max_init, 255, callback)
    cv2.createTrackbar('V Min', 'Trackbars', V_min_init, 255, callback)
    cv2.createTrackbar('V Max', 'Trackbars', V_max_init, 255, callback)

    while True:
        # Obtaining image from drone
        frame = drone.get_frame_read().frame
        if frame is None:
            continue

        # Image rescaling
        frame = cv2.resize(frame, (width, height))
        # Imgage color conversion
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Convierte la imagen de BGR a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Obtaining trackbars value
        h_min = cv2.getTrackbarPos('H Min', 'Trackbars')
        s_min = cv2.getTrackbarPos('S Min', 'Trackbars')
        v_min = cv2.getTrackbarPos('V Min', 'Trackbars')
        h_max = cv2.getTrackbarPos('H Max', 'Trackbars')
        s_max = cv2.getTrackbarPos('S Max', 'Trackbars')
        v_max = cv2.getTrackbarPos('V Max', 'Trackbars')

        #HSV value arrays 
        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        # Aplica desenfoque gaussiano para reducir el ruido
        blurred = cv2.GaussianBlur(hsv, (15, 15), 0)
        mask = cv2.inRange(blurred, lower_hsv, upper_hsv)

        # Erosion and dilation for noise reduction
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Apply mask on original mask
        result = cv2.bitwise_and(frame, frame, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Drawing object contorus and centroid
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
                    lr_vel = 60    
                elif x + w//2 < width // 2 - x_threshold:
                    cv2.putText(frame, f'Objeto a la izquierda', (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    lr_vel = -60
                else: 
                    cv2.putText(frame, f'Objeto en Rango', (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    fb_vel = 0
                    lr_vel = 0
                    ud_vel = 0
                    J_vel  = 0
                    
                #Revisar si el objeto está a la arriba de la imagen
                if y + h//2 > height // 2 + y_threshold:
                    cv2.putText(frame, f'Objeto arriba', (10, 100), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    ud_vel = 60
                    
                    
                elif y + h//2 < height // 2 - y_threshold:
                    cv2.putText(frame, f'Objeto abajo', (10, 100), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    ud_vel = -60

                else: 
                    cv2.putText(frame, f'Objeto en Rango', (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                                (0, 255, 0), 2)
                    fb_vel = 0
                    lr_vel = 0
                    ud_vel = 0
                    J_vel  = 0
                
            #Dibujar líneas de referencia 
            cv2.line(frame, (width//2 - x_threshold, 0), (width//2 - x_threshold, height),  (255, 0, 0), 3)
            cv2.line(frame, (width//2 + x_threshold, 0), (width//2 + x_threshold, height),  (255, 0, 0), 3)
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

        key = cv2.waitKey(50) & 0xFF

        if key == ord('q'):
            clean_exit()
            break

        if key == ord('t') and drone.get_battery() > 15:
            if flying:
                pass
            else:
                drone.takeoff()
                flying = True

        if key == ord('l'):
            if flying:
                drone.land()
                time.sleep(5)
                flying = False

        
        vels = [lr_vel, fb_vel, ud_vel, J_vel]
        
        if prev != vels and flying: 
            drone.send_rc_control(lr_vel, fb_vel, ud_vel, J_vel)

        prev = vels



def main():
    try:
        control()
    except KeyboardInterrupt:
        clean_exit()

if __name__ == '__main__':
    main()

