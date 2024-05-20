This reposatory is for the safekeeping of the code used by Tugaspace in the Cansat competition of 2024 in Azores
Made by yours trully Flower

**Guide**

Esp32 
- TugaSpace2024Primarymission 
Raspberry Pi
- bbt.py
- launcher.sh
- pythonkill.sh
- cron log 
Flower's comments and yappin and Special thanks


## Esp32 code
# TugaSpace2024Primarymission - 
  This code was originaly made by **Professor Miguel**, only modified by **Flower**(me) to implement the multithreading for 2 way Serial comm with the Raspberry Pi, and optimisse some structs and correct some minor errors in the memory dump and data loss betwhen comms trough anttena.<br>
  - **thread 1** : main c++ code for sensor reading <br>
  
  - **thread 2** : sensor data sending via Serial and antenna and safeguard protocol compatability
  - - when the esp32 turns on it receives it's last reading values from the Raspbery Pi and if it detects that the received values are different from it's reference values, the esp32 updates it's reference values to match the last reading.<br>
  
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
- **Process 2**(camera_protocol)       : This protocol uses the **piCamera library** with the **implemented camera** on the Tugaspace satelite to take and save the photo **every 3 seconds** on the Raspberry Pi's internal memory, so after retrival the images can be inputed on the **Tugaspace's AI** protocol to calculate the **fire's risk** and **safe areas** for the fireman to escape.<br>
This protocol also puts the **pressure, temperature and altitude values** gathered by the bmp_Adafruit sensores in the photo.<br>
  
- **Process 3**(safeguard protocol)    : This protocol is a **last resourt protocol** implemented in case the Raspberry Pi detects that the **esp32 stoped working** and **starts using the backup bmp_Adafruit** to save the **Pressure, Temperature and Altitude** in the data file used in the Serial comm **to prevent complete data loss**
  - When it detects that the esp32 came back online, it sends the last reading so that the esp32 can update it's reference values for it's calculations <<in the first run the last reading is set to 0 to match the esp32's reference values>><br>
                                                                  
      CSV file created as a test by the bbt.py code
![image](https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/5c40626b-d289-4812-984d-4504ab2178ed)

### launcher.sh                - 
This **Shell script code** was completely made by Flower(me again), the code is implemented as a **start-up protocol** to **launch the 
python code**(bbt.py) imediatly when the **Raspberry Pi turns on** and launches the OS.<br>
The script is run with **Shell admin privelege**.

### pythonkill.sh              - 
This Shell script is used for **debuging purposes** and made by Flower(ayo its me again), the code when ran **stops the python code 
bbt.py and the launcher script**, so the Raspberry Pi codes can be modified and the data can be transfered and analised.

### crontab log                - 
Used the **daemon crontab log function** to save the **log of all the actions made in the Raspberry Pi** and save in a log dir

## Flower's comments
  Hello everyone, I'm Flower the Tugaspace's programmer.<br>
  Almost all of the final code was saddly lost and due to not saving the files in other locations except on the Raspberry Pi's internal memory and the esp32 (a bad practice but I was on a tight schedule) it was lost in the fall where the Raspberry Pi was broken in half and the micro Sd card was broken.<br>
  I'm in the process of trying to decript the Sd card and finding scraps of the codes to recreate it, right now I was able to recover the launcher.sh script from the Sd card, and I recreated the bbt.py and the using a old version  I created  and the pythonkill.sh from my Raspberry Pi for testing.<br>
  But I still have a little hope I will be able to recover more things from that broken and corrupted Sd card.<br>
  Overall I loved this competition , it was like nothing I've ever done before, I was able to test my codding skills and meet amazing people and have fun<br>
  A special thanks to professor Miguel from the agrupamento de escolas da lousã for keeping me in place when I'm stressing and losing my mind and to orbisat for making me have fun and turning this into an unforgetable experience

