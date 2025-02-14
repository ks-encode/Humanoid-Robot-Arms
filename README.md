So turns out that the CAD is too big to upload so here is a drive link, downlaod it and put it somewhere safe before I remove it from my account:
https://drive.google.com/drive/folders/1mSpQHzyITcN2FqfBqUYI6n7HmdWliPov?usp=sharing It also has the main report with all the info!

**How to turn robot on without injury**  
1 - Plug in microncontroller to USB 2.0 or 3.0 on a computer  
2 - Wait 0.5 seconds  
3 - Turn on main power cable  

**How to turn robot off without injury**  
1 - Turn off main power cable  
2 - **wait 3 seconds!** as there is still power in the transformer  
3 - unplug USB  

**Using digital twin**
1-Open solidworks and click run macro (running V8)
2-if failed to connect:
    A - Check plugged in  
    B - Check com port matches,if not then change it by editing the macro  
    C - Check no other programs are using the com port (e.g. arduino serial monitor)  
    D - Turn off main robot power, wait 3 seconds, then unplug and plug the USB back in,
        sometimes in the event of a crashed system the usb connection will be corrupted.  
3 - Click program virtually to allow you to move the robot around and save positions  
    A - IMPORTANT when moving a part you must click on the part then wait until the mouse wheel stops spinning, else the wrong part will move  
4 - use the drop down to select to save either the current position in the model, or turn on the main robot power to save the real robot position  
5 - click run program and stand back!  

Can use live feedback to move the model around live, but be very careful as it moves at max speed. Press program virtually first then live control.

**CK**
I feel like he doesnt know a lot about the project, but is chill to work with and will find you resources. So do some research to get a good idea of what is reasonable then present it to him (-:

**TODO**
wire other side of left hand to teensy, mirror what is on the right side  
Write in left hand connections into teensy code  
calibrate force sensors, finger angle sensors and PID for left hand  
There are some palm FSR sensors with wiring spots on the PCB you could wire if you like  

**Ideas for capstone projects**
See recomendations section in theb report for key ideas   

Software
1 - integrate with ros, or improve digital twin  
Electronic/mechatronic  
1 - Replace large servos, fine tune hand control and finger setup to get a stronger and more accurate robot  
2 - improve sensing capabilities such as on hands  
3 - integrate camera (tricky)  
Mechancial  
4 - inverse kinematics - doesnt seam big enough with for a capstone  
5 - improve finger design (could add abduction) - smooth out tendons routing, increase force, increase accuracy, add BLDC motors  
