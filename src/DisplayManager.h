#ifndef __DisplayManager_h
#define __DisplayManager_h

#include <Adafruit_GFX_RK.h>
#include <Adafruit_SSD1306_RK.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define NUMFLAKES 10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
static const unsigned char PROGMEM logo_bmp[] =
    {0b00000000, 0b11000000,
     0b00000001, 0b11000000,
     0b00000001, 0b11000000,
     0b00000011, 0b11100000,
     0b11110011, 0b11100000,
     0b11111110, 0b11111000,
     0b01111110, 0b11111111,
     0b00110011, 0b10011111,
     0b00011111, 0b11111100,
     0b00001101, 0b01110000,
     0b00011011, 0b10100000,
     0b00111111, 0b11100000,
     0b00111111, 0b11110000,
     0b01111100, 0b11110000,
     0b01110000, 0b01110000,
     0b00000000, 0b00110000};

#define XPOS 0 // Indexes into the 'icons' array in function below
#define YPOS 1
#define DELTAY 2

//CWD--
class DisplayManager
{
    public:
        DisplayManager(unsigned long refreshInterval, bool fullDisplayTestOn = false);
        ~DisplayManager();
        void update(String text);

        // CWD-- drawing test funcs
        void testdrawline();
        void testdrawrect(void);
        void testfillrect(void);
        void testdrawcircle(void);
        void testfillcircle(void);
        void testdrawroundrect(void);
        void testfillroundrect(void);
        void testdrawtriangle(void);
        void testfilltriangle(void);
        void testdrawchar(void);
        void testdrawstyles(void);
        void testscrolltext(void);
        void testdrawbitmap(void);
        void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h);
        void runScreenTests();

    private:
        Adafruit_SSD1306 display;
        bool blnFullDisplayTestOn = false;
        unsigned long lastScreenUpdate;
        unsigned long ulScreenRefreshRate = 1000000;
};

#endif // def(__DisplayManager_h)
