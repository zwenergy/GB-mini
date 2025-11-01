// This is a simple proof of concept for a program running on the RP2040
// and creating the actual display data for the Pokemon mini.
// The program contained in "rpfb.h" runs on the Pokemon mini and pulls
// the data from the "RP2040 framebuffer" into the LCD controller
// GDRAM. After this copying, the Pokemon mini gets the button states
// and sends it over to the RP2040 cart (at the address 0x5000).
// No sound for now.

#include "pico/stdlib.h"
#include <string.h>

#include "rpfb.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/flash.h"

#include "pico/multicore.h"

#include "oe.pio.h"
#include "pushData.pio.h"
#include "hale.pio.h"
#include "lale.pio.h"

#include "writecheck.pio.h"
#include "writecheck_addr.pio.h"

// Game Boy ROM.
#include "gb_rom.h"

// Include the GB emulator.
uint8_t audio_read( uint16_t addr );
void audio_write( uint16_t addr, uint8_t val );
#define ENABLE_LCD 1
#define ENABLE_SOUND 1
#include "minigb_apu/minigb_apu.h"
#include "peanut_gb.h"

// For centering the image.
#define PMYSKIP 8
#define PMXOFFSET 8

#define SRAMSIZE 32768

// Address in the save game.
#define FLASHADDR ( PICO_FLASH_SIZE_BYTES - SRAMSIZE )

// Save game frame delay.
#define SAVERAMFRAMES (60 * 3)

// GB stuff.
uint8_t rom_bank0[ 16384 ];
static uint8_t ram[ SRAMSIZE ];
static int LCDLineBusy = 0;
uint32_t doingSave = 0;

struct gb_priv {
    uint32_t lcd_line_hashes[LCD_HEIGHT];
    uint dma_pixel_buffer_chan;
};

static struct gb_priv gb_priv = { 0 };
static uint8_t lineBuffer[ LCD_WIDTH ];

// Audio buffer.
audio_sample_t audioBuffer[ 1100 ];

// Audio sample period in microseconds.
#define AUDIOSAMPLEPER_US (( 1000000 / AUDIO_SAMPLE_RATE ) )

// Packed framebuffer (which is copied from the cart)
#define FBOFFSET 0x4000

// Rumble address
#define RUMBLE (rom + 0x4300)

// Rumble on.
#define RUMBLEON *( (volatile uint8_t*) RUMBLE ) = 1

// Rumble off.
#define RUMBLEOFF *( (volatile uint8_t*) RUMBLE ) = 0

// Audio
#define AUDIO *( (volatile uint8_t*) ( rom + 0x4301 ) )
#define AUDIOSTEPS 32

// Audio volume.
#define AUDIO_VOL *( (volatile uint8_t*) ( rom + 0x4302 ) )

// Use rumble? We do not differentiate here by MBC, so using the rumble
// feature with non-rumble games can cause rumble when no rumble is
// intended.
#define USE_RUMBLE

// FRAMEBLEND defines the number of colors and the number of frames
// blended together to create these colors.
// FRAMEBLEND == 0: B/W, 2 colors, no frames being blended.
// FRAMEBLEND == 1: 3 colors, 2 frames being blended.
// etc.
// The PM display refreshes at roughly 75 Hz, FRAMEBLEND == 1 will reduce
// this effectively to the half and so on.
#define FRAMEBLEND 1
volatile unsigned frameBlendCnt = 0;

// PIO-related variables.
PIO pioWE_g;
uint sm_we_g, sm_we_addr_g;

// PM buttons.
volatile uint32_t pmKeys = 0;

// PM FRAME DONE notify. Is set to 1 when a "complete" frame including
// frame blending was drawn.
volatile uint32_t frameDone = 0;

// TRIPLEFB enables triple frame buffer to avoid screen tearing.
// The "frameDone" notify can also be used to sync to reduce latency 
// and do not use the triple FB.
#define TRIPLEFB

#ifdef TRIPLEFB
#define FBDEPTH 3
volatile uint32_t readPtr = 0;
volatile uint32_t writePtr = 1;
uint64_t frameTimeUS = 16742;
#else
#define FBDEPTH 1
#endif

// "Unpacked" framebuffer
uint8_t fbFull[ FBDEPTH ][ 96 * 64];


//////// PM2040 FB COPY SPECIFIC STUFF END //////////////

// GB Emulator functions.
uint8_t gbReadByteROM( struct gb_s *gb, const uint_fast32_t addr )
{
  if( addr < sizeof( rom_bank0 ) )
    return rom_bank0[addr];

  return rom_gb[ addr ];
}

uint8_t gbReadByteRAM( struct gb_s *gb, const uint_fast32_t addr )
{
  return ram[ addr ];
}

void gbWriteByteRAM( struct gb_s *gb, const uint_fast32_t addr, const uint8_t val )
{
  ram[ addr ] = val;
  
  #ifdef USE_RUMBLE
  // Enable Rumble?
  if ( addr & 0b1000 ) {
    RUMBLEON;
  } else {
    RUMBLEON;
  }
  #endif
}

void gbError( struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t val )
{
  abort();
}

void gbRumble( struct gb_s*, const uint_fast32_t rumble ) {
  if ( rumble ) {
    RUMBLEON;
  } else {
    RUMBLEOFF;
  }
}

void LCDLine( struct gb_s *gb, const uint8_t pixels[ LCD_WIDTH ], const uint_fast8_t line )
{
  // Crop top and bottom to fit 0.5 scaling.  
  int lineSkipped = ( (int) line ) - PMYSKIP;
  
  // Do downscaling.
  if ( lineSkipped >= 0 && lineSkipped < ( 64 * 2 ) ) {
    // Skip every second line and just store it.
    if ( lineSkipped & 1 ) {
      unsigned xPM = 0;
      unsigned curFB = writePtr;
      
      // Do steps by 2 for downscaling by 2.
      for ( unsigned xGB = 0; xGB < LCD_WIDTH; ( xGB += 2 ) ) {
        uint8_t tmp = ( pixels[ xGB ] + pixels[ xGB + 1 ] 
                        + lineBuffer[ xGB ] + lineBuffer[ xGB + 1 ] ) >> 2;
        fbFull[ curFB ][ xPM + PMXOFFSET + ( lineSkipped >> 1 ) * 96 ] = tmp & 0b00000011;
        ++xPM;
      }
    } else {
      // Store into linebuffer.
      memcpy( lineBuffer, pixels, LCD_WIDTH );
    }
    
  } else if ( lineSkipped == 128 ) {
    // Frame done.
  }
}

// GB audio stuff.
static struct minigb_apu_ctx apu;

uint8_t audio_read(uint16_t addr)
{
  return minigb_apu_audio_read(&apu, addr);
}

void audio_write(uint16_t addr, uint8_t val)
{
  minigb_apu_audio_write(&apu, addr, val);
}

void audio_callback(void *ptr, uint8_t *data, int len)
{
  minigb_apu_audio_callback(&apu, (void *)data);
}



// We don't use the Flash cache.
#define XIP_CACHE   0x10000000
#define XIP_NOCACHE 0x13000000
#define XIP_NOCACHE_OFFSET (XIP_NOCACHE - XIP_CACHE)

// Pin Definitions.
#define A0A10 0
#define A1A11 1
#define A2A12 2
#define A3A13 3
#define A4A14 4
#define A5A15 5
#define A6A16 6
#define A7A17 7
#define A8A18 8
#define A9A19 9
#define A20 10

#define D0 17
#define D1 18
#define D2 19
#define D3 20
#define D4 21
#define D5 22
#define D6 23
#define D7 24

#define HALE 11
#define LALE 12
#define WE 13
#define OE 14
#define CS 15

// Audio stuff.
uint32_t sampleCnt = 0;
audio_sample_t maxVal = 0;
audio_sample_t audioScale = 1;

// Actual program running and filling the FB.
void __not_in_flash_func( doGB )() {
  // Clear audio.
  AUDIO = 0;
  
  // Set volume to max.
  AUDIO_VOL = 3;
  
  // Clear screen.
  for ( unsigned b = 0; b < FRAMEBLEND; ++b ) {
    for ( unsigned y = 0; y < 64; ++y ) {
     for ( unsigned x = 0; x < 96; ++x ) {
       fbFull[ b ][ y * 96 + x ] = 0;
     }
    }
  }
  
  static struct gb_s gb;
  enum gb_init_error_e ret;
  
  // Init GB.
  memcpy( rom_bank0, rom_gb, sizeof( rom_bank0 ) );
  ret = gb_init( &gb, &gbReadByteROM, &gbReadByteRAM,
    &gbWriteByteRAM, &gbRumble, &gbError, &gb_priv );


  if ( ret != GB_INIT_NO_ERROR ) {
    // Loop infinite.
    while ( 1 ) {};
  }
  
  // Restore RAM from Flash.
  // Offset addr after the RAM (XIP_BASE).
  uint8_t* addr = (uint8_t*) ( XIP_BASE + FLASHADDR );
  
  // Go over byte-wise.
  // TODO: This is stupid, just read out 32b chunks.
  for ( int i = 0; i < SRAMSIZE; ++i ) {
    ram[ i ] = *addr;
    ++addr;
  }
  
  // Audio init.
  minigb_apu_audio_init( &apu );

  // GB display function.
  gb_init_lcd( &gb, &LCDLine );
  
  // Enable frame skip.
  // gb.direct.frame_skip = 1;
  
  uint64_t frameStart = time_us_64();
  uint32_t saveGameCnt = 0;
  uint32_t saveGameDone = 0;
  
  while( 1 ) {
    gb.gb_frame = 0;
    
    // Advance write pointer?
    uint32_t nextWritePtr = writePtr + 1;
    if ( nextWritePtr == 3 ) {
      nextWritePtr = 0;
    }
    
    if ( nextWritePtr != readPtr ) {
      writePtr = nextWritePtr;
    }

    do {
      __gb_step_cpu( &gb );
      tight_loop_contents();
    } while( gb.gb_frame == 0 );
    
    // Do key input.
    gb.direct.joypad = 0xFF;
    
    if ( pmKeys & 0b00001000 ) {
      // Up.
      gb.direct.joypad_bits.up = 0;
    }
    
    if ( pmKeys & 0b00010000 ) {
      // Down.
      gb.direct.joypad_bits.down = 0;
    }
    
    if ( pmKeys & 0b00100000 ) {
      // Left.
      gb.direct.joypad_bits.left = 0;
    }
    
    if ( pmKeys & 0b01000000 ) {
      // Right.
      gb.direct.joypad_bits.right = 0;
    }
    
    if ( pmKeys & 0b00000100 ) {
      // C.
      gb.direct.joypad_bits.start = 0;
      ++saveGameCnt;
    } else {
      saveGameCnt = 0;
      saveGameDone = 0;
    }
    
    if ( pmKeys & 0b00000001 ) {
      // A.
      gb.direct.joypad_bits.a = 0;
    }
    
    if ( pmKeys & 0b00000010 ) {
      // B.
      gb.direct.joypad_bits.b = 0;
    }
    
    // Check for save game transfer to flash?
    if ( saveGameCnt >= SAVERAMFRAMES && !saveGameDone ) {
      doingSave = 1;
      
      // Save RAM to Flash.
      // Not interrupt-safe.
      uint32_t ints = save_and_disable_interrupts();
    
      // Bytes to be erased have to be a multiple of the sector size.
      // SRAMSIZE is 32768 bytes large.
      // A flash sector is 4096 bytes.
      // So it's naturally a multiple.
      flash_range_erase( FLASHADDR, SRAMSIZE );

      // And write.
      // Bytes to be erased have to be a multiple of the page size.
      // SRAMSIZE is 32768 bytes large.
      // A flash page size is 256 bytes.
      // So it's naturally a multiple.
      flash_range_program( FLASHADDR, ram, SRAMSIZE );
      
      // Restore interrupts.
      restore_interrupts ( ints );
      
      saveGameDone = 1;
      doingSave = 0;
    }
    
    // Wait for frame.
    while ( ( time_us_64() - frameStart ) < frameTimeUS ) {
      tight_loop_contents();
    }

    frameStart = time_us_64();
    
    // Get audio.
    minigb_apu_audio_callback( &apu, audioBuffer );
    sampleCnt = 0;
  }
}

bool __not_in_flash_func( doAudio )( repeating_timer_t* t ) {
  // We will be accessing audio functions in the flash, so don't do 
  // audio stuff when saving is in process.
  if ( doingSave ) {
    return true;
  }
  
  // Check audio.
  audio_sample_t curSample = audioBuffer[ sampleCnt ];
  curSample /= 2;

  if ( abs( curSample ) > maxVal ) {
    maxVal = abs( curSample );
    audioScale = ( maxVal * 2 ) / AUDIOSTEPS;
  }
  
  curSample += maxVal;
  
  // Scale sample.
  audio_sample_t curSampleScaled = curSample / audioScale;
  
  if ( curSampleScaled >= AUDIOSTEPS ) {
    curSampleScaled = AUDIOSTEPS - 1;
  }
  
  
  AUDIO = (uint8_t) curSampleScaled;    
  
  if ( sampleCnt < AUDIO_SAMPLES_TOTAL - 1 ) {
    sampleCnt += 2;
  }
    
  return true;
}

void __not_in_flash_func( handleBuffer )(){
  // Frame buffer setup.
  uint8_t* fb = rom + FBOFFSET;
  
  // Do audio callback.
  static repeating_timer_t t;
  add_repeating_timer_us( AUDIOSAMPLEPER_US, doAudio, 0, &t );
  
  while ( 1 ) {    
    #ifdef TRIPLEFB
    // Advance read pointer?
    if ( frameBlendCnt == 0 ) {
      uint32_t nextReadPtr = readPtr + 1;
      if ( nextReadPtr == 3 ) {
        nextReadPtr = 0;
      }
      
      if ( nextReadPtr != writePtr ) {
        readPtr = nextReadPtr;
      }
    }
    #endif
    
    uint8_t* curFB;
    #ifdef TRIPLEFB
    curFB = ( (uint8_t*) fbFull ) + ( readPtr * 96 * 64 );
    #else
    curFB = ( (uint8_t*) fbFull );
    #endif
    
    // Transform the FB.
    for ( unsigned int y = 0; y < 8; ++y ) {
      for ( unsigned int x = 0; x < 96; ++x ) {
        unsigned tmp = 0;
        for ( unsigned int i = 0; i < 8; ++i ) {
          
          unsigned px = curFB[ x + y * 96 * 8 + i * 96 ];
          
          if ( px > frameBlendCnt ) {
            px = 1;
          } else {
            px = 0;
          }
          
          tmp |= px << i;
        }
        
        fb[ x + y * 96 ] = tmp;
      }
    }
    
    if ( frameBlendCnt == FRAMEBLEND ) {
      frameBlendCnt = 0;
      frameDone = 1;
    
      
    } else {
      ++frameBlendCnt;
    }
    
    uint32_t addrData;
    while ( 1 ) {
      if ( !pio_sm_is_rx_fifo_empty( pioWE_g, sm_we_g ) ) {
        // Got a write. Right lower address?
        pmKeys = pio_sm_get( pioWE_g, sm_we_g );
        addrData = pio_sm_get( pioWE_g, sm_we_addr_g );
        
        if ( addrData == 0x000 ) {
          // Fitting lower address.
          break;
        }
        
      }
    }
  }
}

void __not_in_flash_func( doPIOStuff() ) {
  // Set up PIOs.
  
  // OE toggle program.
  PIO pio = pio0;
  uint sm_oe = pio_claim_unused_sm( pio, false );
  uint offset_oe = pio_add_program( pio, &oe_toggle_program );
  
  // Push byte out.
  uint sm_pushData = pio_claim_unused_sm( pio, false );
  uint offset_pushData = pio_add_program( pio, &push_databits_program );
  
  // HALE latching.
  uint sm_hale = pio_claim_unused_sm( pio, false );
  uint offset_hale = pio_add_program( pio, &hale_latch_program );
  
  // LALE latching.
  uint sm_lale = pio_claim_unused_sm( pio, false );
  uint offset_lale = pio_add_program( pio, &lale_latch_program );
  
  
  // Create DMAs.
  int hale_dma = dma_claim_unused_channel( true );
  int lale_addr_dma = dma_claim_unused_channel( true );
  int data_dma = dma_claim_unused_channel( true );
  
  
  // Move high address to LALE SM.
  dma_channel_config c = dma_channel_get_default_config( hale_dma );

  channel_config_set_transfer_data_size( &c, DMA_SIZE_32 );
  channel_config_set_read_increment( &c, false );
  channel_config_set_write_increment( &c, false );
  channel_config_set_dreq( &c, pio_get_dreq( pio, sm_hale, false) );

  dma_channel_configure(
    hale_dma,
    &c,
    &pio->txf[ sm_lale ], // Write to the LALE SM
    &pio->rxf[ sm_hale ],  // Read from HALE RX FIFO
    1,                                          // Halt after each read
    false                                       // Don't start yet
  );
  
  // Move the adress from LALE SM to the third DMA channel.
  c = dma_channel_get_default_config( lale_addr_dma );

  channel_config_set_transfer_data_size( &c, DMA_SIZE_32 );
  channel_config_set_read_increment( &c, false );
  channel_config_set_write_increment( &c, false );
  channel_config_set_dreq( &c, pio_get_dreq( pio, sm_lale, false) );
  
  channel_config_set_chain_to( &c, hale_dma );     // Trigger the HALE channel again when done
  
  

  dma_channel_configure(
    lale_addr_dma,
    &c,
    &dma_hw->ch[ data_dma ].al3_read_addr_trig, // Write to READ_ADDR_TRIG of data channel
    &pio->rxf[ sm_lale ], // Read from LALE RX FIFO
    1,                                          // Halt after each read
    false                                       // Don't start yet
  );
  
  
  // Read the actual data.
  c = dma_channel_get_default_config( data_dma );

  channel_config_set_transfer_data_size( &c, DMA_SIZE_8 );
  channel_config_set_read_increment( &c, false );
  channel_config_set_write_increment( &c, false );
  channel_config_set_chain_to( &c, lale_addr_dma );     // Trigger the LALE channel again when done
  
  // Set to high priority.
  channel_config_set_high_priority( &c, true );

  dma_channel_configure(
    data_dma,
    &c,
    &pio->txf[ sm_pushData ], // Write to the byte push SM
    &rom[0], // Read from ROM array (will be overwritten)
    1,                                          // Halt after each read
    false                                       // Don't start yet
  );
  
  // Start the SMs.
  oe_toggle_program_init( pio, sm_oe, offset_oe, D0, OE );
  push_databits_program_init( pio, sm_pushData, offset_pushData, D0 );
  hale_latch_program_init( pio, sm_hale, offset_hale, A0A10, HALE );
  #ifndef MULTICART
  lale_latch_program_init( pio, sm_lale, offset_lale, A0A10, LALE );
  #else
  lale_latch_menu_program_init( pio, sm_lale, offset_lale, A0A10, LALE );
  #endif
  
  // Push the base address of the array.
  pio_sm_put( pio, sm_lale, ( ( (uint32_t) rom ) ) >> 15 );
  
  // Start the DMA channels.
  dma_start_channel_mask( 1u << hale_dma );
  dma_start_channel_mask( 1u << lale_addr_dma );

  // Now also start the write check PIO.
  PIO pioWE = pio1;
  uint sm_we = pio_claim_unused_sm( pioWE, false );
  uint offset_we = pio_add_program( pioWE, &write_check_program );
  write_check_program_init( pioWE, sm_we, offset_we, D0, WE );
  
  // Start the write addres SM.
  uint sm_we_addr = pio_claim_unused_sm( pioWE, false );
  uint offset_we_addr = pio_add_program( pioWE, &write_check_addr_program );
  write_check_addr_program_init( pioWE, sm_we_addr, offset_we_addr, A0A10, WE );
  
  // Store the PIO variables related for write-handling global.
  pioWE_g = pioWE;
  sm_we_g = sm_we;
  sm_we_addr_g = sm_we_addr;

  // Handle frame buffer copying.
  multicore_launch_core1( handleBuffer );
  
  // Start actual application.
  doGB( pioWE, sm_we, sm_we_addr );
}

int main() {
  
  // Set higher freq.
  sleep_ms(2);
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  sleep_ms(2);
  set_sys_clock_khz(240000, true);
  
  doPIOStuff();
  
  return 0;
}
