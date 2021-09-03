#include "mbed.h"
#include "USBMouse.h"

// de USBDevice::connect gebeurt blocking door USBMouse
// dit gebeurt bij de constructor van USBMouse
// dus de code hangt vóór main() zolang het device niet is geconnecteerd en geconfigureerd door de USB stack
// in USBMouse connect() vervangen door connect(false)
// nu hangt de code niet meer bij de constructor
// mouse.move geeft geen hardfault, de endpoint write wordt keurig afgeblokt zolang
// het device niet geconfigured is -> OK!

// er is toch nog een ongewenst effect :
// bij disconnect blokkeert main op de mouse.move (blocking write in USBDevice.cpp)
// blijft configured=true terwijl disconnect is gebeurd??
// 

Serial pc(USBTX, USBRX); // tx, rx
USBMouse mouse;
DigitalOut myled(LED1);
int i = 0;
int main() {
    pc.printf("Hello World %i!\n\r",i);
    while(1) {
        // pc.putc(pc.getc() + 1); // echo input back to terminal
        myled = 1;
        mouse.move(20, 0);
        wait(0.5);
        myled = 0;
        wait(0.5);
        i++;
    }
}