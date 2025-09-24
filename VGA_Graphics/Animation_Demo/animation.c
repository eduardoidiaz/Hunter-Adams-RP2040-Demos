/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga16_graphics_v2.h"
// Include standard libraries
#include <hardware/timer.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))

// Wall detection
#define hitBottom(b) (b>int2fix15(400))
#define hitTop(b) (b<int2fix15(25))
#define hitLeft(a) (a<int2fix15(10))
#define hitRight(a) (a>int2fix15(630))

// uS per frame
#define FRAME_RATE 33000

// lED PIN
#define LED 25

// GRAVITY FRACTION FOR BALL
#define GRAVITY_FRACTION 0.75

// BOUNCINESS
#define BOUNCINESS 0.9

// SPI configs for the DAC
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Samples per period in sine table
#define SINE_TABLE_SIZE 256

// Sine table and DAC data table
int raw_sin[SINE_TABLE_SIZE] ;
unsigned short dac_data[SINE_TABLE_SIZE] ;
unsigned short* addr_pointer = &dac_data[0] ;

// DAC config bits (channel A, 1x gain, active)
#define DAC_CONFIG_CHAN_A 0b0011000000000000 

// DMA channel vars
int data_chan ;
int ctrl_chan ;

// the color of the balls
char color = WHITE ;

// balls
#define BALL_SPAWN_X 320
#define BALL_SPAWN_Y 60
fix15 ball0_x ;
fix15 ball0_y ;
fix15 ball0_vx ;
fix15 ball0_vy ;
#define BALL_R  2 // BALL RADIUS

// peg on core 0
#define PEG_R 3 // peg radius
#define PEG_OFFSET 40
fix15 peg0_x = int2fix15(BALL_SPAWN_X) ;
fix15 peg0_y = int2fix15(BALL_SPAWN_Y + PEG_OFFSET) ;

// sum of squares for collisions
#define R_SUM_SQ int2fix15((BALL_R + PEG_R) * (BALL_R + PEG_R))

// strings for VGA display
// char curr_num = "Current number of balls being animated: " ;
// char total_fallen = "Total number of balls that have fallen through the board since reset: " ;
// char time = "Time since reset: " ;

static inline void audio_init() {
  // Init/config SPI
  spi_init(SPI_PORT, 20000000) ;
  spi_set_format(SPI_PORT, 16, 0, 0, 0) ;
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI) ;
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Build the sine wave and DAC data tables
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    // 12 bit sine wave, in range [0, 4095]
    raw_sin[i] = (int)(2047 * sin((float)i*6.383f / (float)SINE_TABLE_SIZE) + 2047) ;
    
    // Mask sine wave with config bits into dac_data
    dac_data[i] = DAC_CONFIG_CHAN_A | (raw_sin[i] & 0x0fff) ;

  }
  
  // Claim two DMA channels
  data_chan = dma_claim_unused_channel(true) ;
  ctrl_chan = dma_claim_unused_channel(true) ;

  // Configure the control channel
  // write the starting address of sound data into data channel's read address register
  dma_channel_config ctrl_conf = dma_channel_get_default_config(ctrl_chan) ;
  channel_config_set_transfer_data_size(&ctrl_conf, DMA_SIZE_32) ;
  channel_config_set_read_increment(&ctrl_conf, false) ;
  channel_config_set_write_increment(&ctrl_conf, false) ;
  channel_config_set_chain_to(&ctrl_conf, data_chan) ; //trigger data_chan when done

  dma_channel_configure(
    ctrl_chan, &ctrl_conf, 
    &dma_hw->ch[data_chan].read_addr, // write addr
    &addr_pointer, // read addr
    1, false
  ) ;

  // configure the data channel, to stream data to spi port
  dma_channel_config data_conf = dma_channel_get_default_config(data_chan) ;
  channel_config_set_transfer_data_size(&data_conf, DMA_SIZE_16) ;
  channel_config_set_read_increment(&data_conf, true) ;
  channel_config_set_write_increment(&data_conf, false) ;
  channel_config_set_dreq(&data_conf, spi_get_dreq(SPI_PORT, true)) ; // keep pace wth spi

  dma_channel_configure(
    data_chan, &data_conf, 
    &spi_get_hw(SPI_PORT)->dr,
    dac_data,
    SINE_TABLE_SIZE, 
    false
  ) ;
}

static inline void play_hit_sound() {
  // only play if channels clear
  if (dma_channel_is_busy(data_chan) || dma_channel_is_busy(ctrl_chan)) {
    return ;
  }
  dma_channel_start(ctrl_chan) ;
}

// create a ball (inline makes it run faster)
static inline void spawnBall(fix15* x, fix15* y, fix15* vx, fix15* vy) {
  // Start in center of screen
  *x = int2fix15(BALL_SPAWN_X) ;
  *y = int2fix15(BALL_SPAWN_Y) ;

  // Generate random float between 0.0 and 1.0 for small initial vx
  float random_float = (float)rand() / (float)RAND_MAX ;
  
  // Scale float to range of [-0.2, 0.2]
  float random_vx_float = (random_float * 0.4f) - 0.2f ;
  
  // Convert the float to a fixed-point number and assign it to vx
  *vx = float2fix15(random_vx_float) ;

  // Moving down
  *vy = int2fix15(0) ;
}

// create the grid
static inline void generateBoard() {
  int rows = 16;
  fix15 xi = peg0_x ;
  fix15 yi = peg0_y ;
  fix15 dy = int2fix15(19) ; // the x and y distance from center of top peg
  fix15 dx = int2fix15(38) ;
  fix15 dx_half = int2fix15(19) ;
  fix15 x_new ;
  
  for (int i = 0; i < rows; i++) {
    yi = peg0_y + multfix15(int2fix15(i), dy) ; // move the row down based on the first row

    for (int j = 0; j <= i; j++) {
      if (i % 2 == 0) { // even rows, the logic is ok
        x_new = xi - multfix15(int2fix15(i), dx_half) + multfix15(int2fix15(j), dx) ; // start from left most peg, then draw pegs to the right until 
      }
      else { // odd row
        x_new = xi - multfix15(int2fix15(i), dx_half) + multfix15(int2fix15(j), dx) ; // start from left most peg, then draw pegs to the right until 
      }
      fillCircle(fix2int15(x_new), fix2int15(yi), PEG_R, DARK_GREEN) ; // draws the pegs for every row
    }
  }
}

// Draw the boundaries
static inline void drawArena() {
  drawVLine(10, 10, 600, WHITE) ;   // left 
  drawVLine(630, 10, 600, WHITE) ;  // Right 
  drawHLine(10, 25, 620, WHITE) ;  // Top
  drawHLine(10, 400, 620, WHITE) ;  // Bottom
}

// Detect wallstrikes, update velocity and position
static inline void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y)) {
    *vy = BOUNCINESS * (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  if (hitBottom(*y)) {
    spawnBall(x, y, vx, vy) ; 
  }
  if (hitRight(*x)) {
    *vx = BOUNCINESS * (-*vx) ;
    *x  = (*x - int2fix15(5)) ;
  }
  if (hitLeft(*x)) {
    *vx = (-*vx) ;
    *x  = (*x + int2fix15(5)) ;
  }

  // Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}

static inline void hitPeg(fix15* ball_x, fix15* ball_y, fix15* ball_vx, fix15* ball_vy, fix15* peg_x, fix15* peg_y) {
  // because this is the ball radius + peg radius, 
  // required to see if the x and y distances are less than the collision distance
  // computing difference in position
  fix15 dx = *ball_x - *peg_x ;
  fix15 dy = *ball_y - *peg_y ;
  fix15 dist_sq = multfix15(dx, dx) + multfix15(dy, dy) ;
  // reduces the 
  if (dist_sq < R_SUM_SQ) {
    play_hit_sound() ;
    fix15 distance = sqrtfix(dist_sq) ;
    fix15 normal_x = divfix(dx, distance) ;
    fix15 normal_y = divfix(dy, distance) ;
    fix15 imm_term = (fix15)(-2 * (multfix15(normal_x, *ball_vx) + multfix15(normal_y, *ball_vy))) ;
    
    if (imm_term > 0) {
      
      fix15 move_out_dist = distance + int2fix15(1) ;

      *ball_x = *peg_x + multfix15(normal_x, move_out_dist) ;
      *ball_y = *peg_y + multfix15(normal_y, move_out_dist) ;

      // insert logic to determine if we hit a new peg
      
      *ball_vx = *ball_vx + ( BOUNCINESS * multfix15(normal_x, imm_term) ) ;
      *ball_vy = *ball_vy +( BOUNCINESS * multfix15(normal_y, imm_term) ) ;
     }
  }
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {

        // Toggle on LED
        gpio_put(LED, !gpio_get(LED)) ;

        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 16)) {
          color = (char)user_input ;
        }
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0, animating the ball bouncing on peg
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // Spawn a ball
    spawnBall(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy) ;
    // spawn the board
    generateBoard() ;

    // Generate the text for the screen
    // writeString(char *str) ;
    setCursor(30, 30);
    se


    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;
      
      // erase ball at old position
      fillCircle(fix2int15(ball0_x), fix2int15(ball0_y), BALL_R, BLACK) ;

      // Redraw background and board
      generateBoard() ;
      drawArena() ;

      // Apply gravity to the ball
      ball0_vy = ball0_vy + float2fix15(GRAVITY_FRACTION) ;

      // Update the walls and edges, and handle wall collisions
      wallsAndEdges(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy) ;

      // update ball's position and velocity
      // need to do for every ball, then for every peg... nested for loops???
      hitPeg(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy, &peg0_x, &peg0_y) ;

      // draw the ball at its new position
      fillCircle(fix2int15(ball0_x), fix2int15(ball0_y), BALL_R, color) ;

      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === Core 1 entry
// ========================================
// put user input thread on core1

void core1_entry() {

  // Add serial input thread to core1 scheduler
  pt_add_thread(protothread_serial) ;

  // start scheduler on core1
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  set_sys_clock_khz(150000, true) ;
  // initialize stio
  stdio_init_all() ;

  // setup audio
  audio_init();

  // SETUP GPIO
  gpio_init(LED) ;
  gpio_set_dir(LED, GPIO_OUT) ;
  gpio_put(LED, 0) ;

  // initialize VGA
  initVGA() ;

  // Seed random number gen
  srand(time_us_32()) ;
  
  // get sum of squares for collision detect
  // Sum squares for ball/peg
  //static fix15 r_sum_sq =  int2fix15((ball_r + peg_r) * (ball_r + peg_r));

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  // add threads
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
}
