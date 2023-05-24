import cv2
import numpy as np
import serial
import time
import struct

# Variables for camera matrix
f = 0.00167 # Focal length
px = 1476   # 2952/2 Principal point x
py = 972    # 1944/2 Principal point y

x = 0
y = 0

x_prev = 0
y_prev = 0

x_vel = 0
y_vel = 0

Ad = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
Bd = np.array([[0.005, 0],[0.1, 0],[0, 0.005],[0, 0.1]])
H = np.array([[1, 0, 0, 0],[0, 0, 1, 0]])
kalman = np.array([[0.1412, 0],[0.0932, 0],[0, 0.1412],[0, 0.0932]])
state = np.array([[0],[0],[0],[0]])
alpha = np.array([0])
beta = np.array([0])

def predict(Ad, Bd, state, alpha, beta):
    u = np.array([alpha,beta])
    predicted_states = Ad@state 
    return predicted_states
    
    
def update(state, measurement, H, kalman):
    v = measurement + H@state
    updated_estimate = state + kalman@v
    return updated_estimate
    


def circcirc(x1, y1, r1, x2, y2, r2):
    """
    Find the intersection points of two circles in the plane.
 
    Parameters:
    x1, y1 : float
        The x and y coordinates of the center of the first circle.
    r1 : float
        The radius of the first circle.
    x2, y2 : float
        The x and y coordinates of the center of the second circle.
    r2 : float
        The radius of the second circle.
 
    Returns:
    points : list of tuples
        A list of tuples representing the intersection points.
    """
    d = np.sqrt((x2-x1) ** 2 + (y2 - y1) **2)
    if d > r1 + r2 or d < abs(r1-r2):
        print("It all shit to hell")
        return[]
    a = (r1 **2 - r2 ** 2 + d ** 2) / (2 * d)
    h = np.sqrt(r1**2 - a **2)
    x3 = x1 + a * (x2 - x1) / d
    y3 = y1 + a * (y2 - y1) / d
    
    x4 = x3 - h * (y2-y1)/d
    y4 = y3 + h * (x2-x1)/d
    
    return (x4,y4)
    

def inverseKinematics(alfa, beta):
    R = 12           #Radius top
    r = 5            #radius bottom
    H = 10           #Height from base to plate
    L1 = 6           #Servo lower arm
    L2 = 12          #Servo upper arm
    theta1 = 0 
    theta2 = 0
    theta3 = 0

    y_servo = 5
    z_servo = 0

    zp = np.zeros(3)
    yp = np.zeros(3)
    theta = np.zeros(3)

    y_out = np.zeros(3)
    z_out = np.zeros(3)
    
    servoamount = 3
    
    P_r2 = [12*np.cos(alfa), 3*np.cos(alfa) + 9*np.cos(beta) - 3*3**(1/2)*np.sin(alfa)*np.sin(beta), 3*np.cos(alfa) + 9*np.cos(beta) + 3*3**(1/2)*np.sin(alfa)*np.sin(beta)]
 
    P_r3 = [12*np.sin(alfa) + 10, 10 - 6*3**(1/2)*np.cos(alfa)*np.sin(beta) - 6*np.sin(alfa), 6*3**(1/2)*np.cos(alfa)*np.sin(beta) - 6*np.sin(alfa) + 10]
    
    
    for i in range(servoamount):
              
        z, y = np.real(circcirc(y_servo, z_servo, L1, P_r2[i], P_r3[i], L2))
        
        # Circcirc Fuck you
        
        
        z_out[i] = z
        y_out[i] = y
       
       
        theta[i] = np.arctan2(z_out[i], y_out[i]) + 0.1328
            
        theta[i] = np.rad2deg(theta[i])
        
        if(theta[i] < 0):
            theta[i] = 0
        elif(theta[i] > 45):
            theta[i] = 45
        
    theta[0] += 9 

    theta = np.round(theta)
    return theta

# Function to undistort image
def undistort(image, camera_matrix, dist_coeffs):
    # Get image size
    h, w = image.shape[:2]

    # Create new camera matrix for undistorted image
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0)

    # Generate lookup table for remapping pixels
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), 5)

    # Apply the distortion correction to the image
    undistorted_image = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

    return undistorted_image

# Gaussion blur function
def gaussian_blur(img, kernel_size, sigma):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), sigma)

frame_length = 128

cap = cv2.VideoCapture(0)  # 0 for the first camera module connected
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_length)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_length)

# Funkar med 200 200
#dp = 0.1           # The inverse ratio of the accumulator resolution to the image resolution,     
#minDist = 10000    # The minimum distance between the centers of detected circles
#param1 = 35        # The higher threshold of the two Canny edge detection thresholds used in the Hough transform
#param2 = 15         # The accumulator threshold for circle detection. Only circles with a vote count greater than param2 are detected
#minRadius = 10     # The minimum radius of the circles to be detected
#maxRadius = 18     # The maximum radius of the circles to be detected
#low_threshold = 10
#high_threshold = 30


dp = 0.1           # The inverse ratio of the accumulator resolution to the image resolution,     
minDist = 10000    # The minimum distance between the centers of detected circles
param1 = 50        # The higher threshold of the two Canny edge detection thresholds used in the Hough transform
param2 = 14        # The accumulator threshold for circle detection. Only circles with a vote count greater than param2 are detected
minRadius = 6      # The minimum radius of the circles to be detected
maxRadius = 12     # The maximum radius of the circles to be detected
low_threshold = 10
high_threshold = 30

while True:
    start = time.time()

    
    ret, frame = cap.read()  # ret = True if frame is read correctly
    if not ret:
        break
    
        
    # Undistort the image
    # Define the camera matrix and distortion coefficients
    #K = np.array([[f, 0, px], [0, f, py], [0, 0, 1]])
    #dist_coef = np.array([-0.52081183 , 0.48923578 , 0.00179718  ,0.00270069 ,-0.30610571])

    # Apply undistortion to the image
    #frame = undistort(frame, K, dist_coef)
    #frame = cv2.resize(frame, (200, 200))
    
    # Apply Gaussian blur with a kernel size of 5x5 and sigma value of 0
    #blurred_frame = cv2.GaussianBlur(frame, (21, 21), 0)
    blurred_frame = cv2.GaussianBlur(frame, (11,11), 0)
    
    # Find edges with Canny algorithm
    edges = cv2.Canny(blurred_frame, low_threshold, high_threshold)

    # Define Hough circle parameters
    image = edges       # The input image

    # Find circles using Hough circle transform
    circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, dp, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

    # Draw detected circles on the original image NOT NEEDED LATER
    #
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)


    # Display the original and blurred frames side by side
    cv2.imshow('Original Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
       break

    #print(circles)
    
    # LQR
    if circles is not None:
        
        y_meas = circles[0][0][0]*2/frame_length - 1
        x_meas = circles[0][0][1]*2/frame_length - 1
        
        
        x = x_meas
        y = y_meas
        
        # With velocity
        #state = np.array([[x], [x_dot], [y], [y_dot]])
        elapsed = (time.time() - start)
        x_vel = x-x_prev/elapsed
        y_vel = y-y_prev/elapsed
        
        vel_cap = 0.1
        
        print(x_vel)
        print(y_vel)
        
        if(x_vel < -vel_cap):
            x_vel = -vel_cap
        elif(x_vel > vel_cap):
            x_vel = vel_cap
            
        if(y_vel < -vel_cap):
            y_vel = -vel_cap
        elif(y_vel > vel_cap):
            y_vel = vel_cap
         

        #print(x_meas)
        #print(y_meas)

        x_prev = x
        y_prev = y
        
        state = np.array([[x],[x_vel] ,[y],[y_vel]])
        #state = np.array([[x],[0] ,[y],[0]])
        
        tuning_value = 0.2
        K = np.array([[tuning_value, 0.3033, 0, 0],[0, 0, tuning_value, 0.3033]])
        
        #K = np.array([[0.9551, 1.3821, 0, 0],[0, 0, 0.9551, 1.3821]])
        
        # Denna funkar typ
        #K = np.array([[0.2916, 0.2437, 0, 0],[0, 0, 0.2916, 0.2437]])
        
        
        
        # TOO NICE
        #K = np.array([[0.2688, 2.7865, 0, 0],[0, 0, 0.2688, 2.7865]])
        
        
        # Update the estimated states
        #state = update(predict(Ad,Bd,state,alpha,beta),measurement,H,kalman)
        #state[1] = 0
        #state[3] = 0
        
        
        u = K @ state
        alfa = u[0]
        beta = -u[1]
        print("_____")
        #print(state[0])
        #print(state[1])
        #print(state[2])
        #print(state[3])
        #print(alfa)
        #print(beta)

        #alfa = np.deg2rad(0)
        #beta = np.deg2rad(0)

        #start_time = time.time()
        
        plate_angle_max = np.array([0.4])
        plate_angle_min = np.array([-0.4])
        
        if alfa > plate_angle_max:
            alfa = plate_angle_max
        elif alfa < plate_angle_min:
            alfa = plate_angle_min
            
        if beta > plate_angle_max:
            beta = plate_angle_max
        elif beta < plate_angle_min:
            beta = plate_angle_min

        #print(alfa)
        #print(beta)
        
        values = inverseKinematics(alfa, beta)
        
        if __name__ == '__main__':
            ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            ser.reset_input_buffer()
            #while True:
            
            #alfa = np.deg2rad(30/2*np.sin(5*time.time()))
            #beta = np.deg2rad(30/2*np.cos(5*time.time()))

            values = inverseKinematics(alfa, beta)
            #print(values)

            string = "," +str(values[0]) + "," + str(values[1]) + "," + str(values[2]) + "\n"
            string = bytes(string, 'utf-8') 
            ser.write(string)
            #time.sleep(0.5)
            
        
    
    #print(elapsed)


#cap.release()
#cv2.destroyAllWindows()

