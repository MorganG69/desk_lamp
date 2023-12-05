#include <stdint.h>
#include "hal_gpio.h"
#include "hal_timer.h"
#include "hal_chip.h"

#define POWER_LED (PORT_B | 2U)
#define WHITE_PIN (PORT_B | 4U)
#define YELLOW_PIN (PORT_B | 1U)
#define MODE_BUTTON (PORT_B | 3U)
#define BRIGHTNESS_BUTTON (PORT_B | 0U)

#define BUTTON_TIMER TIMER0_CH1
#define PWM_TIMER TIMER1
#define WHITE_PWM TIMER1_CH2
#define YELLOW_PWM TIMER1_CH1

#define NUM_BUTTONS 2U

#define LED_OFF 0U
#define MIN_BRIGHTNESS 63U
#define LOW_BRIGHTNESS 127U
#define MED_BRIGHTNESS 191U
#define MAX_BRIGHTNESS 255U
#define NUM_BRIGHTNESS 4U

#define STATE_OFF 0U
#define STATE_WHITE 1U
#define STATE_YELLOW 2U
#define STATE_BOTH 3U

#define SIG_MODE 0U
#define SIG_BRIGHTNESS 1U

static uint8_t const brightness[NUM_BRIGHTNESS] = {MIN_BRIGHTNESS, LOW_BRIGHTNESS, MED_BRIGHTNESS, MAX_BRIGHTNESS};

static uint8_t buttons[NUM_BUTTONS] = {MODE_BUTTON, BRIGHTNESS_BUTTON};
static uint8_t volatile button_states[NUM_BUTTONS] = {0, 0};
static uint8_t volatile last_button_states[NUM_BUTTONS] = {0, 0};

static volatile uint8_t system_state = STATE_OFF;
static volatile uint8_t brightness_state = 0U;

static void assert_delay(void) {
  for(uint16_t i = 0; i < 20000; i++) {
    /* do nothing */
  }
}

/* stop everything and just flash the power LED */
void assert_failed(void) {
  chip_disable_isrs();
  timer_stop(PWM_TIMER);
  timer_stop(BUTTON_TIMER);
  while(1) {
    assert_delay();
    gpio_toggle(POWER_LED);
  }
}

void off_state(uint8_t sig) {
  if(sig == SIG_MODE) {
    /* set power LED, enable white LED and move to white state */
    gpio_set(POWER_LED);
    timer_compare_value(WHITE_PWM, brightness[brightness_state]);
    system_state = STATE_WHITE;
  } else {
    /* ignore */
  }  
}

void white_state(uint8_t sig) {
  if(sig == SIG_MODE) {
    /* turn off the white LEDs, turn on the yellows and move to yellow state */
    timer_compare_value(WHITE_PWM, LED_OFF);
    timer_compare_value(YELLOW_PWM, brightness[brightness_state]);
    system_state = STATE_YELLOW;
  } else if (sig == SIG_BRIGHTNESS) {
    /* cycle through the brightness levels */
    brightness_state = (brightness_state + 1U) & 0x03U;
    timer_compare_value(WHITE_PWM, brightness[brightness_state]);
  } else {
    /* ignore */
  }
}

void yellow_state(uint8_t sig) {
  if(sig == SIG_MODE) {
    /* turn the white LED back on and advance to the BOTH state */
    timer_compare_value(WHITE_PWM, brightness[brightness_state]);
    system_state = STATE_BOTH;
  } else if (sig == SIG_BRIGHTNESS) {
    /* cycle through the brightness levels */
    brightness_state = (brightness_state + 1U) & 0x03U;
    timer_compare_value(YELLOW_PWM, brightness[brightness_state]);
  } else {
    /* ignore */
  }
}

void both_state(uint8_t sig) {
  if(sig == SIG_MODE) {
    /* turn all LEDs off and move to off state */
    gpio_reset(POWER_LED);
    timer_compare_value(WHITE_PWM, LED_OFF);
    timer_compare_value(YELLOW_PWM, LED_OFF);
    system_state = STATE_OFF;
  } else if (sig == SIG_BRIGHTNESS) {
    /* cycle through the brightness levels */
    brightness_state = (brightness_state + 1U) & 0x03U;
    timer_compare_value(WHITE_PWM, brightness[brightness_state]);
    timer_compare_value(YELLOW_PWM, brightness[brightness_state]);
  } else {
    /* ignore */
  }
}


void run_state_machine(uint8_t sig) {
  switch(system_state) {
    case STATE_OFF:
      off_state(sig);
    break;

    case STATE_WHITE:
      white_state(sig);
    break;

    case STATE_YELLOW:
      yellow_state(sig);
    break;
    
    case STATE_BOTH:
      both_state(sig);
    break;

    default:
      /* invalid state never reach here */
      assert_failed();
    break;
  }
}

ISR(TIMER0_COMPA_vect) {
  for(uint8_t i = 0U; i < NUM_BUTTONS; i++) {
    button_states[i] = gpio_get(buttons[i]);  

    if((button_states[i] == 0U) && (last_button_states[i] == 1U)) {
      run_state_machine(i);
    }

    last_button_states[i] = button_states[i];
  }
}

int main(void) {
  gpio_config_pin(POWER_LED, GPIO_MODE_OUTPUT);
  gpio_config_pin(WHITE_PIN, GPIO_MODE_OUTPUT);
  gpio_config_pin(YELLOW_PIN, GPIO_MODE_OUTPUT);
  
  /* Inputs use built in pull ups */
  gpio_config_pin(MODE_BUTTON, GPIO_MODE_INPUT | GPIO_PULL_UP);
  gpio_config_pin(BRIGHTNESS_BUTTON, GPIO_MODE_INPUT | GPIO_PULL_UP);

  /* 60Hz polling button read */
  timer_init(BUTTON_TIMER, TIMER_MODE_OC, TIMER0_PRESCALER_1024);
  timer_compare_value(BUTTON_TIMER, 0xFFU);
  timer_enable_isr(BUTTON_TIMER, TIMER_ISR_UPDATE);
  timer_start(BUTTON_TIMER);

  /* Dual output PWM to drive LEDS */
  timer_init(PWM_TIMER, TIMER_MODE_PWM, TIMER1_PRESCALER_1);
  timer_compare_value(WHITE_PWM, LED_OFF);
  timer_compare_value(YELLOW_PWM, LED_OFF);
  timer_start(PWM_TIMER);

  chip_enable_isrs();

  while(1) {
  }
}
