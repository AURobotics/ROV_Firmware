# check point 

1) check each motor is working properly individually

- Run setup 
- outputHorizontalThrusters[x] = value;
- Try putting 255 and -255 in value to each motor and x from 0 to 3 and check if they are working properly
- controlHmotors();
- outputVerticalThrusters[x] = value;
- controlVmotors();
- If not, check the motor and the connection      
- check the arrangment A , B ,C ,D  

2) check reading data is working write 

- Run setup
- Read data 
- check if the data is correct
- If not, check the connection and the data
- check that data is added in the right location

3) chech the computation of data 
- Run setup
- enter some data as inputs forces or torques 
- check if the data is computed correctly
- If not, check the computation and the data    

4) send data through the serial port and check the output of the motors 

5) Read data from imu and check if the data is correct

6) check lights 

7) check the PID 
