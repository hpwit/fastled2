#ifndef __INC_CLOCKLESS_BLOCK_ESP8266_H
#define __INC_CLOCKLESS_BLOCK_ESP8266_H

#define FASTLED_HAS_BLOCKLESS 1

#define PORT_MASK4 (((1<<USED_LANES_FIRST)-1) & 0x0000FFFFL) //on dit  que l'ion va en faire 10
#define FIX_BITS(bits)  (   ((bits & 0xE0L) << 17) |  ((bits & 0x1EL)<<1)  | (bits & 0x1L)   )
#define FIX_BITS2(bits)  (  ((bits & 0xFF00L)<<4) | ((bits & 0xE0L) << 15) |  ((bits & 0x1EL)<<1)  | (bits & 0x1L)   )
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
#define UPPER_MASK ((PORT_MASK & (3L<<28)) >> 28)
#define LOWER_MASK (PORT_MASK &  ((1L<<28)-1))
FASTLED_NAMESPACE_BEGIN

#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
extern uint32_t _frame_cnt;
extern uint32_t _retry_cnt;
#endif

template <uint8_t LANES, int FIRST_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = GRB, uint32_t PORT_MASK=0,int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 5>
class InlineBlockClocklessController : public CPixelLEDController<RGB_ORDER, LANES, PORT_MASK> {
    typedef typename FastPin<FIRST_PIN>::port_ptr_t data_ptr_t;
    typedef typename FastPin<FIRST_PIN>::port_t data_t;
    
    data_t mPinMask;
    data_ptr_t mPort;
    CMinWait<WAIT_TIME> mWait;
public:
    virtual int size() { return CLEDController::size() * LANES; }
    
    virtual void showPixels(PixelController<RGB_ORDER, LANES, PORT_MASK> & pixels) {
        // mWait.wait();
        /*uint32_t clocks = */
        int cnt=FASTLED_INTERRUPT_RETRY_COUNT;
        while(!showRGBInternal(pixels) && cnt--) {
            ets_intr_unlock();
#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
            _retry_cnt++;
#endif
            delayMicroseconds(WAIT_TIME * 10);
            ets_intr_lock();
        }
        // #if FASTLED_ALLOW_INTTERUPTS == 0
        // Adjust the timer
        // long microsTaken = CLKS_TO_MICROS(clocks);
        // MS_COUNTER += (1 + (microsTaken / 1000));
        // #endif
        
        // mWait.mark();
    }
    
    template<int PIN> static void initPin() {
        if(PIN >= 0 && PIN <= LAST_PIN) {
            _ESPPIN<PIN, 1<<(PIN & 0xFF)>::setOutput();
            // FastPin<PIN>::setOutput();
        }
    }
    
    virtual void init() {
        // Only supportd on pins 12-15
        // SZG: This probably won't work (check pins definitions in fastpin_esp32)
        /*     void (* funcs[])() ={initPin<12>,initPin<13>,initPin<14>,initPin<15>,initPin<16>,initPin<17>,initPin<18>,initPin<19>,initPin<0>,initPin<2>,initPin<3>,initPin<4>,initPin<5>,initPin<21>,initPin<22>,initPin<23>};
         
         // void (* funcs[])() ={initPin<12>, initPin<13>, initPin<14>, initPin<15>, initPin<4>, initPin<5>};
         
         for (uint8_t i = 0; i < USED_LANES; ++i) {
         funcs[i]();
         }*/
        uint32_t port;
        if(PORT_MASK==0)
            port=PORT_MASK_TOTAL;
        else
            port=PORT_MASK;
        uint32_t porti=port;
        for(int i=0;i<32;i++)
        {
            
            Serial.printf("i:%d port:%ld  result:%d\n",i,porti,porti&0x1);
            if( (porti & 0x1)>0 )
            {
                if(i>27)
                    pinMode(i+4,OUTPUT);
                else
                    pinMode(i,OUTPUT);
                
            }
            porti=porti>>1;
        }
       // gpio_pullup_en(GPIO_NUM_0 );
        //gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLDOWN_ONLY);
        mPinMask = FastPin<FIRST_PIN>::mask();
        mPort = FastPin<FIRST_PIN>::port();
        
        // Serial.print("Mask is "); Serial.println(PORT_MASK);
    }
    
    virtual uint16_t getMaxRefreshRate() const { return 400; }
    
    #if NUM_STRIPS >16
        #pragma message "We have Mode than 16 strips"
    typedef union {
        uint8_t bytes[16];
        uint32_t shorts[8];
        uint32_t raw[2];
    } Lines;
    #else
     #pragma message "We have less or  16 strips"
     typedef union {
     uint8_t bytes[16];
     uint16_t shorts[8];
     uint32_t raw[2];
     } Lines;
     #endif
    
#define ESP_ADJUST 0 // (2*(F_CPU/24000000))
#define ESP_ADJUST2 0
    template<int BITS,int PX> __attribute__ ((always_inline)) inline static void writeBits(register uint32_t & last_mark, register Lines & b, PixelController<RGB_ORDER, LANES, PORT_MASK> &pixels) { // , register uint32_t & b2)  {
        Lines b2=b ;
        //Lines b4=b3;
        // transpose16x1_noinline(b.bytes,b2.shorts);
         #if NUM_STRIPS >16
        transpose24x1_noinline(b.bytes,b2.shorts);
        #else
             transpose16x1_noinline(b.bytes,b2.shorts);
         #endif
        // transpose24x1(b.bytes,b2.shorts);
        //transpose8x1_noinline(b3.bytes,b4.bytes);
        //uint32_t port=0;
        register uint8_t d = pixels.template getd<PX>(pixels);
        register uint8_t scale = pixels.template getscale<PX>(pixels);
        //if(PORT_MASK==0)
        //  port=PORT_MASK_TOTAL ;
        //else
        // port=PORT_MASK;
        
        for(register byte i = 0; i < 8; i++) { //USED_LANES
            uint32_t nword=0;
            uint32_t maskK=PORT_MASK;
            // uint32_t ert=0;
            
            //ert=(uint32_t)(~b2.shorts[7-i]);
            
            //*FastPin<FIRST_PIN>::sport() = port ;
#ifdef FastMask
#pragma message "We have Set the fastMask"
            // nword=FastMask((~b2.shorts[7-i]));
#else
            uint32_t m=b2.shorts[7-i];
            for (register uint32_t  j=0;j< LANES;j++)
                // while(j<LANES)
            {
                // j++;
                // if((uint32_t)(~b2.shorts[7-i]) & (1L<<j))    ///if(ert & 1) //on a une 1 a ecrire`
                if(!(m & 1))
                {
                    nword+=    ((maskK&(maskK-1) ) ^  maskK); //&( (~b2.shorts[7-i]) & (1L<<j))) ;
                    
                }
                
                //ert=ert>>1;
                maskK=maskK&(maskK-1);
                m=m>>1;
            }
            
#endif
            
            //uint32_t nword = fixbit((uint32_t)(~b2.shorts[7-i]),PORT_MASK,(uint32_t)LANES);
            while((__clock_cycles() - last_mark) < (T1+T2+T3));
            last_mark = __clock_cycles();
            *FastPin<FIRST_PIN>::sport() = LOWER_MASK ;
            
            // REG_WRITE(GPIO_OUT_W1TS_REG,LOWER_MASK);
            REG_WRITE(GPIO_OUT1_W1TS_REG,UPPER_MASK);
            while((__clock_cycles() - last_mark) < (T1-6));
            // *FastPin<FIRST_PIN>::cport() = nword;
            //*FastPin<FIRST_PIN>::cport() = lowernword;
#ifdef FastMask
#pragma message "We have Set the fastMask"
            nword=FastMask((~b2.shorts[7-i]));
#endif
            
            uint32_t lowernword=(uint32_t)(nword &  ((1L<<28)-1));
            uint8_t uppernword=(uint8_t)((nword & (3L<<28)) >> 28);
            //*FastPin<FIRST_PIN>::cport() = LOWER_MASK;
            //REG_WRITE(GPIO_OUT_W1TC_REG,lowernword);
            *FastPin<FIRST_PIN>::cport() = lowernword;
            REG_WRITE(GPIO_OUT1_W1TC_REG,uppernword);
            
            if(i<LANES)
                b.bytes[i] = pixels.template loadAndScale<PX>(pixels,i,d,scale);
            if (i+8<LANES)
                b.bytes[i+8] = pixels.template loadAndScale<PX>(pixels,i+8,d,scale);
            
            while((__clock_cycles() - last_mark) < (T1+T2));
            *FastPin<FIRST_PIN>::cport() = LOWER_MASK;
            //  REG_WRITE(GPIO_OUT_W1TC_REG,LOWER_MASK);
            REG_WRITE(GPIO_OUT1_W1TC_REG,UPPER_MASK);
            // *FastPin<FIRST_PIN>::cport() = port;
            //*FastPin<FIRST_PIN>::cport() = (PORT_MASK << REAL_FIRST_PIN ) | ( PORT_MASK2 );
            
            if((16+i)<LANES)
                b.bytes[16+i] = pixels.template loadAndScale<PX>(pixels,i+16,d,scale);
            
            
            
            //  b.bytes[i] = pixels.template loadAndScale<PX>(pixels,i,d,scale);
        }
        
        
        
        
    }
    
    // This method is made static to force making register Y available to use for data on AVR - if the method is non-static, then
    // gcc will use register Y for the this pointer.
    static uint32_t showRGBInternal(PixelController<RGB_ORDER, LANES, PORT_MASK> &allpixels) {
        
        // Setup the pixel controller and load/scale the first byte
        Lines b0;//,b1;
        
        for(int i = 0; i < LANES; i++) {
            //if(i<USED_LANES_FIRST)
            b0.bytes[i] = allpixels.loadAndScale0(i);
            //if (i<USED_LANES_SECOND)
            //  b1.bytes[i] = allpixels.loadAndScale0(i+8);
            
        }
        
        
        
        
        allpixels.preStepFirstByteDithering();
        
        ets_intr_lock();
        
        
       // portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
      //  portENTER_CRITICAL(&mux);
        
        uint32_t _start = __clock_cycles();
        uint32_t last_mark = _start;
        
          //  while((__clock_cycles() - last_mark) < (NS(500000)));
        
        
        _start = __clock_cycles();
        last_mark = _start;
        
        while(allpixels.has(1)) {
            // Write first byte, read next byte
            writeBits<16+XTRA0,1>(last_mark, b0,allpixels);
            
            // Write second byte, read 3rd byte
            writeBits<16+XTRA0,2>(last_mark, b0, allpixels);
            allpixels.advanceData();
            
            // Write third byte
            writeBits<16+XTRA0,0>(last_mark, b0,allpixels);
            
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
        
        ets_intr_unlock();
#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
        _frame_cnt++;
#endif
       // portEXIT_CRITICAL(&mux);
        return __clock_cycles() - _start;
    }
};

FASTLED_NAMESPACE_END
#endif
