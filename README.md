# icemaker
This repo contains a firmware for an ice machine,plenty of picture of the inside, and a reversed schematic

# Model
![image](https://github.com/user-attachments/assets/f469274f-db64-415c-8d3e-6b161a9eade4)

this is my ice machine : https://www.gonser.ch/fr/machine-a-glacons-noir-12-kg/24-h/a-12385/

I can't find a brand name and the only thing to identify it is the model : 600375 
and the scoop motor marking : 50TYZ-E

# About
I made theses because the scoop motor axis rusted and was jammed. I took it apart and cleaned it but it added play/backlash which prevented the "normal operation" as it was now working but the original firmware assumed the motor would always bouce back correctly when it reach an endswitch. This is a unipolar motor, the same kind that make the microwave table turn and they usually start up in a random direction. You can make them go the way you want if you "springload" them against the endswitch but this is not super good (the scoop has a crack now due to fatigue from this).
So I removed the unknow MCU chip from fremontmicro and replaced it by an arduino (Sparkfun pro micro)
and made a working firmware after reverse the PCB and get what pin of the MCU was driving what.

# state of the firmware

the firmware is currently useable with the following feature :
- Ice full detection
- motor wrong way recovery
- full ice making cycle

what dont work : 
- empty water detection ( for some reason it works for the IR signal detection but not for the water conduction)
- size selection ( the button is not correctly detected and the firmware see that the power button was pressed)
- on button, altough it make the on led change state I didnt bother making it stop the machine
- air temperature sensor (the code should work but I could not find the correct beta and gave up for now)

(my machine is working fine but this is provided as his with no garatee that it wont make your machine explode or worse :)
 # picture
 look inside the folder named picture :)
 
 # schematic
 look for schematic.pdf
 ![image](https://github.com/user-attachments/assets/7ef22dcd-f650-4ae2-be0d-54e533d22d60)
 (there might be some mistake left but it was good enough to get most of the firmware working)
 (I didnt bother documenting how exactly the triac are wired as it's not adding much)
