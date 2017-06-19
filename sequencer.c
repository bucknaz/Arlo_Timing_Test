

#include "simpletools.h"                      // Include simpletools
#include "sequencer.h"

static volatile int t, dt, cog;               // Global var for cogs to share
static unsigned int stack[40 + 25];           // Stack vars for other cog

void sequencer(void *par);                 


int sequencer_start()
{
  sequencer_stop();
  cog = 1 + cogstart(sequencer, NULL, stack, sizeof(stack));
}

void sequencer_stop()
{
  if(cog)
  {
    cogstop(cog -1);
    cog = 0;
  }    
}

int sequencer_get()
{
  return t;
}

void sequencer_reset()
{
  t = 0;
}

void sequencer_set(int newTime)
{
  t = newTime;
}

// Function runs in another cog
void sequencer(void *par)                      
{
  dt = CLKFREQ/1000;
  int ticks = CNT;
  while(1)                                   
  {
    waitcnt(ticks+=dt);                              
    t++;                                     
/*
    if(t % 250 == 0 ) qs++;
    if(t % 500 == 0 ) hs++;
    if(t % 750 == 0 ) tqs++;
    if(t % 1000 == 0 ) s++;
*/
  }                            
}
