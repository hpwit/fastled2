#ifndef __INC_CLOCKLESS_BLOCK_ESP8266_H
#define __INC_CLOCKLESS_BLOCK_ESP8266_H

#define FASTLED_HAS_BLOCKLESS 1

#define PORT_MASK4 (((1<<USED_LANES_FIRST)-1) & 0x0000FFFFL) //on dit  que l'ion va en faire 10
#define FIX_BITS(bits)  (   ((bits & 0xE0L) << 17) |  ((bits & 0x1FL)<<2)  )
#define FIX_BITS2(bits)  (  ((bits & 0xFF00L)<<4) | ((bits & 0xE0L) << 15) |  ((bits & 0x1EL)<<1) | (bits & 0x1L)   )
#define  PORT_MASK3 (((1<<USED_LANES_SECOND )-1) & 0x0000FFFFL)
#define PORT_MASK2 FIX_BITS(PORT_MASK3)
#define MIN(X,Y) (((X)<(Y)) ? (X):(Y))
//attempted to use other/more pins for the port
// #define USED_LANES (MIN(LANES,33))
#define USED_LANES (MIN(LANES,16))
#define USED_LANES_FIRST ( (LANES)>8 ? (8) :(LANES) )
#define USED_LANES_SECOND ( (LANES)>8 ? (LANES-8) :(0) )
#define REAL_FIRST_PIN 12
// #define LAST_PIN (12 + USED_LANES - 1)
#define LAST_PIN 35
#define SUBLANES 1<<LANES
//#define PORT_MASK_TOTAL PORT_MAX_T
//#define PORT_MASK PORT_MAX_T
#define PORT_MASK_TOTAL  ( FIX_BITS2((1<<LANES)-1) )
#include "driver/gpio.h"
#include "driver/rtc_io.h"
FASTLED_NAMESPACE_BEGIN

#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
extern uint32_t _frame_cnt;
extern uint32_t _retry_cnt;
#endif

template <uint8_t LANES, int FIRST_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = GRB, uint32_t PORT_MASK=0,int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 50>
class InlineBlockClocklessController : public CPixelLEDController<RGB_ORDER, LANES, PORT_MASK> {
    typedef typename FastPin<FIRST_PIN>::port_ptr_t data_ptr_t;
    typedef typename FastPin<FIRST_PIN>::port_t data_t;
    PixelController<RGB_ORDER,LANES,PORT_MASK> *local_pixels  = NULL;
    data_t mPinMask;
    data_ptr_t mPort;
    CMinWait<WAIT_TIME> mWait;
    TaskHandle_t handleRGBTHandle = NULL;
    TaskHandle_t userRBGHandle = NULL;
public:
    virtual int size() { return CLEDController::size() * LANES; }
    
    
    
    void static handleRGB(void *pvParameters)
    {
        
        InlineBlockClocklessController* c = static_cast<InlineBlockClocklessController*>(pvParameters);
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
        // -- Run forever...
        for(;;) {
            // -- Wait for the trigger
            ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
            
            // -- Do the show (synchronously)
            // noInterrupts();
            // FastLED.delay(10);
            //  Serial.printf("on affiche");
            c->showRGBInternal(*(c->local_pixels));
            
            xTaskNotifyGive(c->userRBGHandle);
        }
    }
    
    
    virtual void showPixels(PixelController<RGB_ORDER, LANES, PORT_MASK> & pixels) {
        
        local_pixels=&pixels;
        
        showRGBInternal(pixels);
        
        uint32_t port;
        
        int cnt=FASTLED_INTERRUPT_RETRY_COUNT;
        
        
        
    }
    
    template<int PIN> static void initPin() {
        if(PIN >= 0 && PIN <= LAST_PIN) {
            _ESPPIN<PIN, 1<<(PIN & 0xFF)>::setOutput();
            // FastPin<PIN>::setOutput();
        }
    }
    
    virtual void init() {
        
        
        
        uint32_t port;
        if(PORT_MASK==0)
            port=PORT_MASK_TOTAL;
        else
            port=PORT_MASK;
        uint32_t porti=port;
        
        REG_WRITE(GPIO_ENABLE_REG,port);
        
    }
    
    virtual uint16_t getMaxRefreshRate() const { return 400; }
    
    typedef union {
        uint8_t bytes[16];
        uint16_t shorts[8];
        uint32_t raw[2];
    } Lines;
    
#define ESP_ADJUST 0 // (2*(F_CPU/24000000))
#define ESP_ADJUST2 0
    template<int BITS,int PX> __attribute__ ((always_inline)) inline static void writeBits(register uint32_t & last_mark, register Lines & b, PixelController<RGB_ORDER, LANES, PORT_MASK> &pixels) { // , register uint32_t & b2)  {
        Lines b2=b ;
        //Lines b4=b3;
        transpose16x1_noinline(b.bytes,b2.shorts);
        //transpose8x1_noinline(b3.bytes,b4.bytes);
        uint32_t port=0;
        register uint8_t d = pixels.template getd<PX>(pixels);
        register uint8_t scale = pixels.template getscale<PX>(pixels);
        if(PORT_MASK==0)
            port=PORT_MASK_TOTAL ;
        else
            port=PORT_MASK;
        
        
        
        
        
        
        //last_mark = __clock_cycles();
        for(register uint32_t i = 0; i < 8; i++) { //USED_LANES
            uint32_t nword=0;
            uint32_t maskK=port;
            uint32_t ert=0;
            //    if(PORT_MASK>0)
            for (register uint32_t  j=0;j< LANES;j++)
            {
                if((uint32_t)(~b2.shorts[7-i]) & (1<<j))    ///if(ert & 1) //on a une 1 a ecrire
                {
                    nword= nword |  ((maskK&(maskK-1) ) ^  maskK) ;
                    
                }
                
                //ert=ert>>1;
                maskK=maskK&(maskK-1);
            }
            
            while((__clock_cycles() - last_mark) < (T1+T2+T3));
            last_mark = __clock_cycles();
            
            
            //*FastPin<FIRST_PIN>::sport() = port ;
            
           REG_WRITE(GPIO_OUT_W1TS_REG,port);
            
            while((__clock_cycles() - last_mark) < (T1-6));
            
            REG_WRITE(GPIO_OUT_W1TC_REG,nword);
            //*FastPin<FIRST_PIN>::cport() = nword;
            
            
            if(i<LANES)
                b.bytes[i] = pixels.template loadAndScale<PX>(pixels,i,d,scale);
            if(8+i<LANES)
                b.bytes[i+8] = pixels.template loadAndScale<PX>(pixels,i+8,d,scale);
            
            // GPIO.out_w1tc = nword;
            //__asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;");
            while((__clock_cycles() - last_mark) < (T1+T2));
            //*FastPin<FIRST_PIN>::cport() = port;
            REG_WRITE(GPIO_OUT_W1TC_REG,port);
            
        }
        
        
        
        
        
        
    }
    
    // This method is made static to force making register Y available to use for data on AVR - if the method is non-static, then
    // gcc will use register Y for the this pointer.
    static uint32_t ICACHE_RAM_ATTR showRGBInternal(PixelController<RGB_ORDER, LANES, PORT_MASK> &allpixels) {
        
        
        // Setup the pixel controller and load/scale the first byte
        Lines b0,b1;
        uint32_t port;
        
        
        //uint32_t port;
        if(PORT_MASK==0)
            port=PORT_MASK_TOTAL;
        else
            port=PORT_MASK;
        
        
        for(int i = 0; i < LANES; i++) {
            
            b0.bytes[i] = allpixels.loadAndScale0(i);
            
            
        }
        
        
        
        
        allpixels.preStepFirstByteDithering();
        //ets_intr_lock();
        
        uint32_t _start = __clock_cycles();
        uint32_t last_mark = _start;
        
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);
        
        
        
        while((__clock_cycles() - last_mark) < (NS(500000)));
        _start = __clock_cycles();
        last_mark = _start;
        
        uint32_t porti=PORT_MASK;
        /* for(int i=0;i<32;i++)
        {
            
            //Serial.printf("i:%d port:%ld  result:%d\n",i,porti,porti&0x1);
            if( (porti & 0x1)>0 )
            {
                pinMode(i,OUTPUT);
               // gpio_pad_select_gpio((gpio_num_t)i );
                //PIN_FUNC_SELECT((gpio_num_t)i ,PIN_FUNC_GPIO);//GPIO_PIN_MUX_REG[i], PIN_FUNC_GPIO);
                //gpio_pulldown_en((gpio_num_t)i );
                //gpio_set_direction((gpio_num_t)i, GPIO_MODE_OUTPUT_OD );
                //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[(gpio_num_t)i], PIN_FUNC_GPIO);
                //gpio_set_direction((gpio_num_t)i, GPIO_MODE_OUTPUT);
               // gpio_matrix_out((gpio_num_t)i, signal, 0, 0);
                
            }
            porti=porti>>1;
        }`*/
        
        while(allpixels.has(1)) {
            // Write first byte, read next byte
            //portDISABLE_INTERRUPTS();
            writeBits<16+XTRA0,1>(last_mark, b0,allpixels);
            //last_mark = __clock_cycles();
            // Write second byte, read 3rd byte
            writeBits<16+XTRA0,2>(last_mark, b0, allpixels);
            //last_mark = __clock_cycles();
            allpixels.advanceData();
            
            // Write third byte
            writeBits<16+XTRA0,0>(last_mark, b0,allpixels);
            
            //portENABLE_INTERRUPTS();
#if (FASTLED_ALLOW_INTERRUPTS == 1)
            ets_intr_unlock();
#endif
            
            allpixels.stepDithering();
            
#if (FASTLED_ALLOW_INTERRUPTS == 1)
            ets_intr_lock();
            // if interrupts took longer than 45µs, punt on the current frame
            if((int32_t)(__clock_cycles()-last_mark) > 0) {
                if((int32_t)(__clock_cycles()-last_mark) > (T1+T2+T3+((WAIT_TIME-INTERRUPT_THRESHOLD)*CLKS_PER_US))) { ets_intr_unlock(); return 0; }
            }
#endif
            
        };
        
#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
        _frame_cnt++;
#endif
        porti=PORT_MASK;
       /* for(int i=0;i<32;i++)
        {
            
            //Serial.printf("i:%d port:%ld  result:%d\n",i,porti,porti&0x1);
            if( (porti & 0x1)>0 )
            {
                pinMode(i,INPUT);
               // gpio_pad_select_gpio((gpio_num_t)i );
              // gpio_set_pull_mode((gpio_num_t)i, GPIO_FLOATING);
               //gpio_set_direction((gpio_num_t)i, GPIO_MODE_INPUT );
                
                
            }
            porti=porti>>1;
        }*/
        
        
        
        portEXIT_CRITICAL(&mux);
        //Serial.printf(" on a :%ld\n",__clock_cycles() - _start);
        return __clock_cycles() - _start;
        
    }
    
    
    
    
    
    
};

FASTLED_NAMESPACE_END
#endif
