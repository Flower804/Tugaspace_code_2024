This reposatory is for the safekeeping of the code used by Tugaspace in the [Cansat competition of 2024](https://www.esero.pt/projetos-escolares/2023-2024/cansatpt) in Azores
Made by yours trully Flower

**Guide** <br>
- Esp32 
  - TugaSpace2024Primarymission
 
- Raspberry Pi
  - bbt.py
  - launcher.sh
  - pythonkill.sh
  - cron log
- AI explanation
- Materials
- Flower's comments and yappin and Special thanks
- Links
<br>

<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/664a5fd4-9d02-4af1-86c9-9aa12a2bee85" width=150>



## Esp32 code
- # TugaSpace2024Primarymission - 
  This code was originaly made by **Professor Miguel**, only modified by **Flower**([me](https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/0bceb48a-e981-446c-becf-8ed3dd1fcf03)) to implement the [multithreading](https://www.freertos.org/Documentation/RTOS_book.html) for 2 way Serial comm with the Raspberry Pi, and optimisse some structs and correct some minor errors in the memory dump and data loss betwhen comms trough anttena.<br>
   - **thread 1** : main c++ code for sensor reading <br>
  
   - **thread 2** : sensor data sending via Serial and antenna and safeguard protocol compatability
     - when the esp32 turns on it receives it's last reading values from the Raspbery Pi and if it detects that the received values are different from it's reference values, the esp32 updates it's reference values to match the last reading.<br>
  
The code reads values of the **Pressure,Temperature, Latitude, Longitude, GPSaltitude, Horizontal velocity and time DFRobot and TinyGPS sensores** <br>
  And using the values it gathers to **calculate its Altitude and time** between readings <br>
  And then sending the data trough it's **anteena** for the data to be **captured in the ground station** and **sending the data trough Serial for the Raspberry Pi**<br>
<p> 
  
## Raspberry Pi codes
- # bbt.py(Raspberry Pi code)   - 
  This code was completely made by Flower(me :D), the code **uses [multiprocessing](https://docs.python.org/3/library/multiprocessing.html) to run 3 processes simultaneously**. <br>
  - **Process 1**(communication_protocol): This protocol **reads the Serial** comm betwhen the esp32 and the Raspberry Pi 
to **receive and save all the data gathered** by the esp32 on the Raspberry 
Pi's internal memory as a **csv file** so we can **prevent dataloss if the 
groundstation losses communication** with the esp32.<br>
  - **Process 2**(camera_protocol)       : This protocol uses the **piCamera library** with the **implemented camera** on the Tugaspace satelite to take and save the photo **every 3 seconds** on the Raspberry Pi's internal memory, so after retrival the images can be inputed on the **Tugaspace's AI** protocol to calculate the **fire's risk** and **safe areas** for the fireman to escape.<br>
This protocol also puts the **pressure, temperature and altitude values** gathered by the bmp_Adafruit sensores in the photo.<br>
  
  - **Process 3**(safeguard protocol)    : This protocol is a **last resourt protocol** implemented in case the Raspberry Pi detects that the **esp32 stoped working** and **starts using the backup bmp_Adafruit** to save the **Pressure, Temperature and Altitude** in the data file used in the Serial comm **to prevent complete data loss**
    - When it detects that the esp32 came back online, it sends the last reading so that the esp32 can update it's reference values for it's calculations
    -  > in the first run when the esp32 asks for the last values the Raspberry Pi sends the default values <br>
                                                                  
      CSV file created as a test by the bbt.py code
![image](https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/5c40626b-d289-4812-984d-4504ab2178ed)

- ### launcher.sh                - 
This **Shell script code** was completely made by Flower(me again), the code is implemented as a **start-up protocol** to **launch the 
python code**(bbt.py) imediatly when the **Raspberry Pi turns on** and launches the OS.<br>
The script is run with **Shell admin privelege**.

- ### pythonkill.sh              - 
This Shell script is used for **debuging purposes** and made by Flower(ayo its me again), the code when ran **stops the python code 
bbt.py and the launcher script**, so the Raspberry Pi codes can be modified and the data can be transfered and analised.

- ### crontab log                - 
Used the **daemon crontab log function** to save the **log of all the actions made in the Raspberry Pi** and save in a log dir
## AI explanation
<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/507483c4-9dc8-4aa4-98b9-f05ecd60dfa1" width=600>

## Materiais
- ### Cansat
  - esp32 wroom 32u wi-fi
  - Raspberry Pi Zero 2
  - MPU-6500
  - Veml7700
  - GT-U7 gps
  - DFrobot BMP 388
  - ds18b20
  - APC220
  - Bateria 602535 3.7v 600mah
  - RASPBERRY-PI RPI-6MM LENS
  - Ra 02 Lora
<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/4b3e0cf3-4a15-4964-80b5-7a03363479c4" width=150>
<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/510399f8-33f8-42ce-9799-449af62c569e" width=150>
<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/0393dfde-89f6-4798-938e-e0a6bdbaf4f0" width=150>

- ### Ground Station
  - 3 ground stations
    - 3x esp32 c6 wroom 1
    - 3x Ra 02 Lora
    - Yagi Uda anttena
    - 2x omnidirecional anttenas
<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/0eeeff6e-b3dd-43d9-9b6b-29e71f301e3e" width=200>
<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/07030e50-a907-4e26-9938-917822230791" width=362>

## Flower's comments
  Hello everyone, I'm Flower the Tugaspace's programmer.<br>
  Almost all of the final code was saddly lost and due to not saving the files in other locations except on the Raspberry Pi's internal memory and the esp32 (a bad practice but I was on a tight schedule) it was lost in the fall where the Raspberry Pi was broken in half and the micro Sd card was broken.<br>
  I'm in the process of trying to decript the Sd card and finding scraps of the codes to recreate it, right now I was able to recover the launcher.sh script from the Sd card, and I recreated the bbt.py and the using a old version  I created  and the pythonkill.sh from my Raspberry Pi for testing.<br>
  But I still have a little hope I will be able to recover more things from that broken and corrupted Sd card.<br>
  Overall I loved this competition , it was like nothing I've ever done before, I was able to test my codding skills and meet amazing people and have fun<br>
  A special thanks to professor Miguel from the agrupamento de escolas da lousã for keeping me in place when I'm stressing and losing my mind and to orbisat for making me have fun and turning this into an unforgetable experience
<br>
## Thank You
![IMG-20240430-WA0000](https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/2338f4b9-1693-46ea-9dce-5acd76486b1c)

# Links
Tugaspace
- [GitHub](https://github.com/TugaSpace)
- [Intagram](https://www.instagram.com/tuga.space.cansat/)
- [Facebook](https://www.facebook.com/people/Tuga-SpaceTeam/pfbid02gAVXFdknUKmTnEWCLY2dwU7gth6EGuVqS172sN5ZeFfWGMRoak1ZSCv8JEX9aXGXl/)
- Youtube
  - [Tugaspace 2024](https://youtu.be/kA14SlcFrzM?si=Yo5XSjHVyRpjZ8U3)
  - [Tugaspace final - 2023](https://youtu.be/N3DgtH6w_Gg?si=islSonAAbx6NKa-j)
- [Trevim](https://www.trevim.pt/tag/tugaspace/)
- [AGLOUSA](https://escolas.aglousa.com/2024/05/08/os-tugaspace-na-final-11o-cansat/)
- [GOfundme](https://www.gofundme.com/f/help-a-team-looking-for-money-for-a-competition?member=25147017&sharetype=teams&utm_campaign=p_na+share-sheet&utm_medium=copy_link&utm_source=customer)

Flower
- [GitHub](https://github.com/Flower804)
- [Instagram](https://www.instagram.com/gab_moita/)
- Email: gabrielmoita34@gmail.com

<img src="https://github.com/Flower804/Tugaspace_code_2024/assets/146494346/6b9ff970-a79f-4966-9509-3fb6e4f36b5d" width=200>

