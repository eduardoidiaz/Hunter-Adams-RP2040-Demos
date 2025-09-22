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
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 33000

// the color of the balls
char color = WHITE ;

// balls
fix15 ball0_x ;
fix15 ball0_y ;
fix15 ball0_vx ;
fix15 ball0_vy ;
short ball_r = 4 ; // radius

// peg on core 0
fix15 peg0_x = int2fix15(320) ;
fix15 peg0_y = int2fix15(240) ;

// create a ball (inline makes it run faster)
inline void spawnBall(fix15* x, fix15* y, fix15* vx, fix15* vy) {
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(120) ;

  // Generate random float between 0.0 and 1.0 for small initial vx
  float random_float = (float)rand() / (float)RAND_MAX ;
  
  // Scale float to range of [-0.5, 0.5]
  float random_vx_float = random_float - 0.5 ;
  
  // Convert the float to a fixed-point number and assign it to vx
  *vx = float2fix15(random_vx_float) ;

  // Moving down
  *vy = int2fix15(0) ;
}

// create a peg
inline void spawnPegs() {
  fillCircle(fix2int15(peg0_x), fix2int15(peg0_y), 6, DARK_GREEN) ;
  // to generate a filled circle
}

// Draw the boundaries
inline void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect wallstrikes, update velocity and position
inline void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Reverse direction if we've hit a wall
  if (hitTop(*y)) {
    *vy = (-*vy) ;
    *y  = (*y + int2fix15(5)) ;
  }
  if (hitBottom(*y)) {
    spawnBall(x, y, vx, vy) ; 
  }
  if (hitRight(*x)) {
    *vx = (-*vx) ;
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


void hitPeg(fix15* ball_x, fix15* ball_y, fix15* ball_vx, fix15* ball_vy, fix15* peg_x, fix15* peg_y) {
  // because this is the ball radius + peg radius (6 + 4 = 10), 
  // required to see if the x and y distances are less than the collision distance
  static const fix15 collision_dist_sq = float2fix15(100.0) ; 
  // computing difference in position
  fix15 dx = *ball_x - *peg_x ;
  fix15 dy = *ball_y - *peg_y ;
  fix15 dist_sq = multfix15(dx, dx) + multfix15(dy, dy) ;
  // reduces the 
  if (dist_sq < collision_dist_sq) {
     fix15 distance = sqrtfix(dist_sq) ;
     fix15 normal_x = divfix(dx, distance) ;
     fix15 normal_y = divfix(dy, distance) ;
     fix15 imm_term = (fix15)(-2 * (multfix15(normal_x, *ball_vx) + multfix15(normal_y, *ball_vy))) ;
     
     if (imm_term > 0) {
       *ball_vx = *ball_vx + multfix15(normal_x, imm_term) ;
       *ball_vy = *ball_vy + multfix15(normal_y, imm_term) ;

       fix15 move_out_dist = distance + int2fix15(1) ; // need to randomize for the binomial distribution right or left

       *ball_x = *peg_x + multfix15(normal_x, move_out_dist) ;
       *ball_y = *peg_y + multfix15(normal_y, move_out_dist) ;
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

    // Spawn a boid
    spawnBall(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy) ;
    spawnPegs() ;

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;
      
      // erase ball at old position
      fillCircle(fix2int15(ball0_x), fix2int15(ball0_y), 4, BLACK) ;

      // Redraw background
      spawnPegs() ;
      drawArena() ;

      // Apply gravity to the ball
      ball0_vy = ball0_vy + float2fix15(0.1) ;

      // Update the walls and edges, and handle wall collisions
      wallsAndEdges(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy) ;

      // update ball's position and velocity
      hitPeg(&ball0_x, &ball0_y, &ball0_vx, &ball0_vy, &peg0_x, &peg0_y) ;

      // draw the ball at its new position
      fillCircle(fix2int15(ball0_x), fix2int15(ball0_y), 4, color) ;

      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  set_sys_clock_khz(150000, true) ;
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // Seed random number gen
  srand(time_us_32()) ;

  // start core 1
  //multicore_reset_core1();
  //multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
}
