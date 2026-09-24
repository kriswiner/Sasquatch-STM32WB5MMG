Arduino sketch to read PDM data from the ICS41350 PDM microphone and use in-line FFT analysis to print amplitude in 16 frequency bands between 0 and 8 kHz on the serial monitor every 2 seconds.

Connect Sasquatch PDM CLK to ICS41350 CLK, Sasquatch PDM DAT to ICS41350 DATA, ICS41350 SELECT (L/R) to GND for DAT1 (or 3V0 for DAT2, and 3V0 and GND from Sasquatch to the ICS41350 breakout board.

<img width="2137" height="1424" alt="IMG_1098" src="https://github.com/user-attachments/assets/9ac1aca2-07bf-4188-bebb-6895dd686765" />

*Typical output:*

Frequency_Hz	Amplitude  
RMS	60.12  
125.0	450.0  
343.8	1070.8  
562.5	2367.6  
781.2	3058.4  
1000.0	437.1  
1218.7	116.0  
1437.5	76.7  
1656.2	34.6  
1875.0	85.9  
2093.8	33.9  
2312.5	25.5  
2531.3	20.4  
2750.0	69.4  
2968.8	78.0  
3187.5	134.0  
3406.3	105.9  
3625.0	65.7  
3843.8	65.1  
4062.5	67.2  
4281.3	29.4  
4500.0	36.5  
4718.8	20.3  
4937.5	23.2  
5156.3	11.7  
5375.0	10.3  
5593.8	7.9  
5812.5	8.4  
6031.3	8.6  
6250.0	12.3  
6468.8	7.5  
6687.5	11.7  
6906.3	9.5  
Peak_Hz	781.2  

ICS41350 breakout board design is available on [OSHPark](https://oshpark.com/shared_projects/N40xu0vJ).


