#include "mbed.h"
#include "USBSerial.h"

// de USBDevice::connect gebeurt blocking door USBMouse
// dit gebeurt bij de constructor van USBMouse
// dus de code hangt vóór main() zolang het device niet is geconnecteerd en geconfigureerd door de USB stack
// in USBMouse connect() vervangen door connect(false)
// nu hangt de code niet meer bij de constructor
// mouse.move geeft geen hardfault, de endpoint write wordt keurig afgeblokt zolang
// het device niet geconfigured is -> OK!

// er is toch nog een ongewenst effect :
// bij disconnect blokkeert main op de mouse.move (blocking write in USBDevice.cpp)
// USBDevice.cpp aangepast, zodat device.suspended wordt gebruikt om een blocking write te vermijden
// configured blijft true, want we kunnen geen disconnect detecteren, enkel een suspend
// 

Serial pc(USBTX, USBRX); // tx, rx
//Virtual serial port over USB
USBSerial usbSerial;
DigitalOut myled(LED1);

static void settingsChangedCallback (int baud, int bits, int parity, int stop);

int i = 0;
int myBaud = 0;

int main() {
    pc.printf("Hello World %i!\n\r",i);
    usbSerial.attach(settingsChangedCallback);

    while(1) {
        //usbSerial.printf("%i - I am a virtual serial port @ %i baud \n",i, myBaud);
        // pc.putc(pc.getc() + 1); // echo input back to terminal
        myled = 1;
        wait(0.5);
        myled = 0;
        wait(0.5);
        i++;
        while (usbSerial.available())
        {
            usbSerial.printf("got : %c\n", usbSerial.getc());
        }
    }
}


static void settingsChangedCallback (int baud, int bits, int parity, int stop)
{
    // this can never work, because settingsChangedCallback is called in ISR context
    // and printf is doing a blocking write on an endpoint!
    /*
    if (usbSerial.connected()) {
        usbSerial.printf("baud = %i, %i data, %i parity, %i stop\n",baud, bits,parity, stop);
    }
    */
   myBaud = baud;
}
