#pragma once

//***********************************************************************
// leds.h - LED bar graph display and battery voltage monitor.
//
//  LED_Bar(color, count) - light 'count' LEDs red (0) or green (1)
//  battery_monitor()     - rolling-average ADC read; drives LED bar
//                          except when modes 4/5 own the display
//***********************************************************************

void LED_Bar(int LED_color, int LED_count)
{
  if(LED_color == 0)
  {
    for(int i=0; i<LED_count; i++)
    {
      digitalWrite((RED_LED1+(4*i)),   HIGH);
      digitalWrite((GREEN_LED1+(4*i)), LOW);
    }
    for(int i=LED_count; i<8; i++)
    {
      digitalWrite((RED_LED1+(4*i)),   LOW);
      digitalWrite((GREEN_LED1+(4*i)), LOW);
    }
  }
  else
  {
    for(int i=0; i<LED_count; i++)
    {
      digitalWrite((GREEN_LED1+(4*i)), HIGH);
      digitalWrite((RED_LED1+(4*i)),   LOW);
    }
    for(int i=LED_count; i<8; i++)
    {
      digitalWrite((GREEN_LED1+(4*i)), LOW);
      digitalWrite((RED_LED1+(4*i)),   LOW);
    }
  }
}


void battery_monitor()
{
  // sliding-window average over 50 samples
  batt_voltage_sum -= batt_voltage_array[batt_voltage_index];
  batt_voltage_array[batt_voltage_index] = map(analogRead(BATT_VOLTAGE), 0, 1023, 0, 1497);
  batt_voltage_sum += batt_voltage_array[batt_voltage_index];
  batt_voltage_index++;
  if(batt_voltage_index > 49) batt_voltage_index = 0;

  batt_voltage = batt_voltage_sum / 50;

  // 2S LiPo: 6.4 V (640) = empty, 8.4 V (840) = full → 1–8 LEDs
  batt_LEDs = map(constrain(batt_voltage, 640, 840), 640, 840, 1, 8);

  // modes 4 (leg lift) and 5 (autonomous) own the LED bar; skip here
  if(mode != 4 && mode != 5)
  {
    if(batt_voltage > 700) LED_Bar(1, batt_LEDs);  // green  > 3.5 V/cell
    else                   LED_Bar(0, batt_LEDs);  // red   <= 3.5 V/cell
  }
}
