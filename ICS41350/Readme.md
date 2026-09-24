Arduino sketch to read PDM data from the ICS41350 PDM microphone with the Sasquatch development board and use in-line FFT analysis to print amplitude in 16 frequency bands between 0 and 8 kHz on the serial monitor every 2 seconds.

Connect Sasquatch PDM CLK to ICS41350 CLK, Sasquatch PDM DAT to ICS41350 DATA, ICS41350 SELECT (L/R) to GND for DAT1 (or 3V0 for DAT2, and 3V0 and GND from Sasquatch to the ICS41350 breakout board.

<img width="2137" height="1424" alt="IMG_1098" src="https://github.com/user-attachments/assets/9ac1aca2-07bf-4188-bebb-6895dd686765" />

*Typical output:*

Frequency_Hz	Amplitude  
RMS	77.96  
125.0	343.4  
359.4	1701.0  
609.4	4078.5  
859.4	403.8  
1109.4	345.7  
1359.4	651.0  
1609.4	312.6  
1859.4	136.8  
2109.4	75.4  
2359.4	90.8  
2609.4	355.9  
2859.4	234.0  
3109.4	99.1  
3359.4	157.9  
3609.4	193.2  
3859.4	377.6  
4109.4	106.1  
4359.4	62.3  
4609.4	66.7  
4859.4	70.8  
5109.4	50.7  
5359.4	31.4  
5609.4	28.1  
5859.4	34.3  
6109.4	21.8  
6359.4	42.4  
6609.4	79.8  
6859.4	100.8  
7109.4	68.8  
7359.4	25.9  
7609.4	7.8  
7859.4	6.8  
Peak_Hz	609.4    

ICS41350 breakout board design is available on [OSHPark](https://oshpark.com/shared_projects/N40xu0vJ).


