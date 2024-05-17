This reposatory is for the safekeeping of the code used by Tugaspace in the Cansat competition of 2024 in Azores
Made by yours trully Flower
Esp32
- TugaSpace2024Primarymission
Raspberry Pi
- bbt.py
- launcher.sh
- scriptkill.sh
- cron log

Esp32 code-------------------------------------------------------------------------------------------------------------------------------------------------
TugaSpace2024Primarymission - This code was originaly made by Professor Miguel, only modified by Flower(me) to implement the Serial 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;comm with the Raspberry Pi and optimisse some 
                              structs and correct some minor errors in the memory 
                              &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;dump and data loss betwhen comms trough anttena.<br>
                          &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;The code reads values of the Pressure,Temperature, Latitude, Longitude, GPSaltitude, Horizontal velocity and time 
                          &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;using Adafruit, MPU, LoRa, SPI,                                     DFRobot and TinyGPS sensores <br>
                          &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;And using the values it gathers to calculate its Altitude, time between readings <br>
                          &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;And then sending the data trough it's anteena for the data to be captured in the ground station and sending the 
                          &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;data trough Serial for the Raspberry Pi<br>
<p>                              
Raspberry Pi codes--------------------------------------------------------------------------------------------------------------------------
bbt.py(Raspberry Pi code)   - This code was completely made by Flower(me :D), the code uses multiprocessing to run 3 processes simultaneously. <br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Process 1(communication_protocol): This protocol reads the Serial comm betwhen the esp32 and the Raspberry Pi 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;to receive and save all the data gathered by the esp32 on the Raspberry 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Pi's internal memory as a csv file so we can prevent dataloss if the 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;groundstation losses
                                                                communication with the esp32.<br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Process 2(camera_protocol)       : This protocol piCamera library with the implemented camera on the Tugaspace 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;satelite to take and save the photo 
every 3
                                                                seconds on the Raspberry Pi's 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;internal memory, so after retrival the images can be inputed on the 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Tugaspace's AI protocol to calculate the fire's risk and safe areas for the 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;fireman to escape.<br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;This protocol also puts the pressure, temperature and altitude values gathered by 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;the bmp_Adafruit sensores.<br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;Process 3(safeguard protocol)    : This protocol is a lastresourt protocol implemented in case the Raspberry Pi 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;detects that the esp32 stoped working and uses starts using the backup 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;bmp_Adafruit to save the Pressure, Temperature and Altitude in the data 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;file used in the 
                                                                  Serial comm to prevent complete data loss <br>
                                                                  
                  -------------------------CSV file created as a test by the bbt.py code---------------------------
![image](https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/5c40626b-d289-4812-984d-4504ab2178ed)

launcher.sh                - This Shell script code was completely made by Flower(me again), the code is implemented as a start-up protocol so launch the 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;python code(bbt.py) imediatly when the Raspberry Pi turns on and launches the OS.<br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;The script is run with Shell admin privelege.

scriptkill.sh              - This Shell script is used for debuging purposes and made by Flower(ayo its me again), the code when ran stops the python code 
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;bbt.py and the launcher script, so the Raspberry Pi codes can be modified and the data can be transfered and analised.

crontab log                - Used the daemon crontab log function to save the log of all the actions made in the Raspberry Pi and save in a log dir

