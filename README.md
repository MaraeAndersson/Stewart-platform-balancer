# Stewart platform balancer (WIP)
By Martin Andersson and Oliver Moberg

## Description
The stewart-platform-balancer is a 3 Degrees Of Freedom (DOF) stewart platform made with the intention to further our knowledge about control systems, computer vision and filtering. The goal of the project is to have the platform manage to balance a ball on the platform by using a Linear Quadratic Regulator (LQR) to control the three servos controling the orientation of the platform.
 
To determine the x and y position of the ball a wide lens camera was mounted below the transparent acrylic platform which was used in combination with a Raspberry Pi 3 and openCV2 to detect the position of the ball. The Canny edge detection algorithm was used to find the contours of the ball and the Hough circle transform was then to find all the contours that are circles. These algorithms were then tuned to only find the ball placed on the platform. 

 
![alt text]( Stewart_platform_V1.png "Prototype of the stewart platform balancer")
