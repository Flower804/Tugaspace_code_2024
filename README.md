This reposatory is for the safekeeping of the code used by Tugaspace in the Cansat competition of 2024 in Azores
Made by yours trully Flower
Esp32
- TugaSpace2024Primarymission
Raspberry Pi
- bbt.py
- launcher.sh
- scriptkill.sh
- cron log

## Esp32 code
# TugaSpace2024Primarymission - 
  This code was originaly made by **Professor Miguel**, only modified by **Flower**(me) to implement the Serial comm with the Raspberry Pi and optimisse some structs and correct some minor errors in the memory dump and data loss betwhen comms trough anttena.<br>
  The code reads values of the **Pressure,Temperature, Latitude, Longitude, GPSaltitude, Horizontal velocity and time DFRobot and TinyGPS sensores** <br>
  And using the values it gathers to **calculate its Altitude and time** between readings <br>
  And then sending the data trough it's **anteena** for the data to be **captured in the ground station** and **sending the data trough Serial for the Raspberry Pi**<br>
<p> 
  
## Raspberry Pi codes
# bbt.py(Raspberry Pi code)   - 
  This code was completely made by Flower(me :D), the code **uses multiprocessing to run 3 processes simultaneously**. <br>
- **Process 1**(communication_protocol): This protocol **reads the Serial** comm betwhen the esp32 and the Raspberry Pi 
to **receive and save all the data gathered** by the esp32 on the Raspberry 
Pi's internal memory as a **csv file** so we can **prevent dataloss if the 
groundstation losses communication** with the esp32.<br>
- **Process 2**(camera_protocol)       : This protocol uses the** piCamera library** with the **implemented camera** on the Tugaspace satelite to take and save the photo **every 3 seconds** on the Raspberry Pi's internal memory, so after retrival the images can be inputed on the **Tugaspace's AI** protocol to calculate the **fire's risk** and **safe areas** for the fireman to escape.<br>
This protocol also puts the **pressure, temperature and altitude values** gathered by the bmp_Adafruit sensores in the photo.<br>
  
- **Process 3**(safeguard protocol)    : This protocol is a **last resourt protocol** implemented in case the Raspberry Pi detects that the **esp32 stoped working** and **starts using the backup bmp_Adafruit** to save the **Pressure, Temperature and Altitude** in the data file used in the Serial comm **to prevent complete data loss** <br>
                                                                  
      CSV file created as a test by the bbt.py code
![image](https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/5c40626b-d289-4812-984d-4504ab2178ed)

### launcher.sh                - 
This **Shell script code** was completely made by Flower(me again), the code is implemented as a **start-up protocol** to **launch the 
python code**(bbt.py) imediatly when the **Raspberry Pi turns on** and launches the OS.<br>
The script is run with **Shell admin privelege**.

### scriptkill.sh              - 
This Shell script is used for **debuging purposes** and made by Flower(ayo its me again), the code when ran **stops the python code 
bbt.py and the launcher script**, so the Raspberry Pi codes can be modified and the data can be transfered and analised.

### crontab log                - 
Used the **daemon crontab log function** to save the **log of all the actions made in the Raspberry Pi** and save in a log dir

