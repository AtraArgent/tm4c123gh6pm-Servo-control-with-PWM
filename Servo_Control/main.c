#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"

#define PWM_FREQUENCY 55 //55Hz Servo frekansi

int main()
{
    //  Optimizasyon ayarindan bagimsiz olarak derleyicinin bunlari ortadan kaldirmayacaginin
    //garantisi icin "volatile" olarak tanimlanilar.
    //  ui8Adjust degiskeni servonun konumunu ayarlamamiza izin verecektir.
    //83, PWM'de 1.5ms darbe genisligi olusturmak icin merkezi konumdur.
    //Servo kontrol kodunda PWM periyodu 1000'e b�lecegiz. Programlanan frekans
    //55Hz ve periyot 18.2ms oldugudan, bunu 1000'e b�lerek bize 1.8us puls c�z�n�rl�k verir.
    //Bunun 83 ile carpilmasi bize 1.51ms'lik bir darbe genisligi verir. C�z�n�rl�k vs icin
    //diger secimler 1.5ms darbe genisligi �rettikleri s�rece gecerli olacaktir.Say�n�z�n 16
    //bitlik saklayicilara sigdigina dikkat ediniz.
    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;
    volatile uint32_t ui8Adjust;
    ui8Adjust = 83;

    //  islemciyi 40MHz'de kosturalim. PWM mod�l�, sistem saati tarafindan bir b�l�c�n�n
    //�zerinden saglanir ve bu b�l�c�n�n 2 ile 64 arasinda bir deger araligi vardir.
    //B�l�c�y� 64'e ayarlayarak, PWM saati 625Khz'de calistirirlir.
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //  PWM1 ve GPIOD mod�llerini (PD0'daki PWM cikisi icin) ve GPIOF mod�l�n� (PF0 ve PF4'teki
    //LaunchPad d�gmeleri icin) etkinlestirmemiz gerekir.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Port D pin 0 (PD0), modul 1, PWM jenarator 0 icin bir PWM cikis pini olarak yapilandirilmalidir.
    // M1PWM0'�n bu kodda PWM cikisi olarak secilmesinin nedeni Vcc ve GND'ye yakin olmasindan �t�r�d�r.
    //
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    // Port F uzerindeki 0. ve 4. pinleri launchPad uzerindeki sw1 ve sw2 anahtarlarina baglanmistir.
    //Bu pinlere tam erisimi saglayabilmek icin asagidaki kodlar kullanilmistir. Son olarak anahtarlar
    //pull-up olarak ayarlanmisitir.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //  PWM saati, SYSCLK/64't�r. PWM saatini, Load saklayicisina y�klenecek sayiyi belirlemek icin
    //istenen frekansa(55Hz)b�l�n. Ard�ndan, saya� s�f�ra kadar sayd��� i�in 1'i ��kar�n. Modul 1
    //PWM jenerator 0'� alt-sayac olarak yapipilandirin ve sayi degerini y�kleyin.
    ui32PWMClock = SysCtlClockGet()/64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    //  Son olarak PWM ayarlarini yapabilir ve etkinlestirebiliriz. �lk satir darbe genisligini belirler.
    //PWM Load degeri 1000'e b�l�n�r (servo icin minimum c�z�n�rl�g� belirler) ve ayar degeri ile carpilir.
    //Bu sayilar daha fazla veya daha az c�z�n�rl�k saglamak icin degistirilebilir. �kinici ve �c�nc�
    //satirlarda, PWM mod�l 1, jenerat�r 0'�n bir cikis olarak etkinlesitirilmesi ve calistirilmasi ayarlarnir.
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load/1000);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    while(1)
    {
        //  Bu kod SW1�e bas�l�p bas�lmad���n� g�rmek i�in PF4 pinini okuyacakt�r.Bu kod her �al��t�r�ld���nda,
        //alt 1mS s�n�r�na ula�mad��� s�rece ayar de�i�kenini bir azaltacakt�r.Bu say�, merkez ve �st konumlar
        //gibi PWM'nin ��k���n� �l�erek belirlendi.Son sat�r PWM darbe geni�lik yaz�c�s�n� yeni de�erle y�kler.
        //Bu y�k, asenkron olarak ��k��a yap�l�r.
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0x00)
        {
            ui8Adjust--;
            if(ui8Adjust < 26) //90 DERECE �C�CN 56
            {
                ui8Adjust = 26;
            }
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load/1000);
        }
        //Bir sonraki kod, darbe geni�li�ini art�rmak i�in SW2'ye bas�l�p bas�lmad���n� g�rmek i�in PF0 pimini
        //okuyacakt�r. Maksimum s�n�r 2,0 mS'ye ayarlanm��t�r.
        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)==0x00)
                {
                    ui8Adjust++;
                    if(ui8Adjust > 141) //90 DERECE �C�N 111
                    {
                        ui8Adjust = 141;
                    }
                    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load/1000);
                }
        //Bu son �izgi d�ng�n�n h�z�n� belirler. Servo sizin i�in �ok h�zl� veya �ok yava� hareket ederse,
        //say�n�z� istedi�iniz gibi de�i�tirmek i�in �ekinmeyin.
        SysCtlDelay(100000);
    }

}

//void position_servo(uint8_t ui8Degrees, uint32_t ui32Load) // Servo position from 0 to 180 degrees
//{
//    uint8_t ui8Adjust;
//
//    if (ui8Degrees > 180)
//
//    {
//       ui8Adjust = 111;
//    }
//
//    else
//
//    {
//        ui8Adjust = 56.0 + ((float)ui8Degrees * (111.0 - 56.0)) / 180.0;
//    }
//
//    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);

//}
