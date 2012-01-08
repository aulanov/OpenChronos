// *************************************************************************************************
//
//	Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
//	Copyright (C) 2011 Frank Van Hooft
//
//
//	  Redistribution and use in source and binary forms, with or without
//	  modification, are permitted provided that the following conditions
//	  are met:
//
//	    Redistributions of source code must retain the above copyright
//	    notice, this list of conditions and the following disclaimer.
//
//	    Redistributions in binary form must reproduce the above copyright
//	    notice, this list of conditions and the following disclaimer in the
//	    documentation and/or other materials provided with the
//	    distribution.
//
//	    Neither the name of Texas Instruments Incorporated nor the names of
//	    its contributors may be used to endorse or promote products derived
//	    from this software without specific prior written permission.
//
//	  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//	  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//	  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//	  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//	  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//	  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//	  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************
// Altitude measurement functions.
// *************************************************************************************************


// *************************************************************************************************
// Include section

// system
#include "project.h"

#ifdef FEATURE_ALTITUDE

#include "altitude.h"
#include "altitude_math.h"
#include "display.h"
#include "vti_ps.h"
#include "ports.h"
#include "timer.h"

#include "menu.h"
#include "user.h"
#ifdef CONFIG_VARIO
# include "vario.h"
#endif

#if defined(CONFIG_ALTITUDE_UNIT_FEET) && defined(CONFIG_ALTITUDE_UNIT_METERS)
#error "You cannot define both CONFIG_ALTITUDE_UNIT_FEET and CONFIG_ALTITUDE_UNIT_METERS"
#endif
#if !defined(CONFIG_ALTITUDE_UNIT_FEET) && defined(CONFIG_METRIC_ONLY)
#define CONFIG_ALTITUDE_UNIT_METERS
#endif
#if !defined(CONFIG_ALTITUDE_UNIT_FEET) && !defined(CONFIG_ALTITUDE_UNIT_METERS)
#define CONFIG_ALTITUDE_UNIT_SETTABLE
#endif

struct alt sAlt;

extern void (*fptr_lcd_function_line1)(u8 line, u8 update);
extern void (*fptr_lcd_function_line2)(u8 line, u8 update);

// Global flag for pressure sensor initialisation status
extern u8 ps_ok;

#ifdef CONFIG_ALTITUDE_SKYDIVING
void sx_altitude_skydiving(u8 line) {
	// Ignore.
}

void mx_altitude_skydiving(u8 line) {
	// Ignore.
}

void nx_altitude_skydiving(u8 line) {
	// Ignore.
}

void display_altitude_skydiving(u8 line, u8 mode) {
	// Ignore.
}

u8 display_update_altitude_skydiving(void) {
	return 0;
}

const struct menu menu_L2_Altitude =
{
	sx_altitude_skydiving,
	mx_altitude_skydiving,
	nx_altitude_skydiving,
	display_altitude_skydiving,
	display_update_altitude_skydiving,
};
#endif

// *************************************************************************************************
// @fn          reset_altitude_measurement
// @brief       Reset altitude measurement.
// @param       none
// @return      none
// *************************************************************************************************
void reset_altitude_measurement(void)
{
	sAlt.state = MENU_ITEM_NOT_VISIBLE;
	sAlt.timeout = 0;
	sAlt.altitude = 0;
	sAlt.mode = ALTITUDE_REGULAR;

	// Pressure sensor ok?
	if (ps_ok)
	{
		// Do single conversion
		start_altitude_measurement();
		stop_altitude_measurement();
	}
}

// *************************************************************************************************
// @fn          is_altitude_measurement
// @brief       Altitude measurement check
// @param       none
// @return      u8		1=Measurement ongoing, 0=measurement off
// *************************************************************************************************
u8 is_altitude_measurement(void)
{
	return ((sAlt.state == MENU_ITEM_VISIBLE) && (sAlt.timeout > 0));
}


// *************************************************************************************************
// @fn          start_altitude_measurement
// @brief       Start altitude measurement
// @param       none
// @return      none
// *************************************************************************************************
void start_altitude_measurement(void)
{
	// Show warning if pressure sensor was not initialised properly
	if (!ps_ok)
	{
		display_chars(LCD_SEG_L1_2_0, (u8*)"ERR", SEG_ON);
		return;
	}

	// Start altitude measurement if timeout has elapsed
	if (sAlt.timeout == 0)
	{
		// Enable DRDY IRQ on rising edge
		PS_INT_IFG &= ~PS_INT_PIN;
		PS_INT_IE |= PS_INT_PIN;

		// Start pressure sensor
		ps_start();

		// Set timeout counter only if sensor status was OK
		sAlt.timeout = ALTITUDE_MEASUREMENT_TIMEOUT;

		// Get updated altitude
		while((PS_INT_IN & PS_INT_PIN) == 0);
		do_altitude_measurement(FILTER_OFF);
	}
}


// *************************************************************************************************
// @fn          stop_altitude_measurement
// @brief       Stop altitude measurement
// @param       none
// @return      none
// *************************************************************************************************
void stop_altitude_measurement(void)
{
	// Return if pressure sensor was not initialised properly
	if (!ps_ok) return;

	// Stop pressure sensor
	ps_stop();

	// Disable DRDY IRQ
	PS_INT_IE  &= ~PS_INT_PIN;
	PS_INT_IFG &= ~PS_INT_PIN;

	// Clear timeout counter
	sAlt.timeout = 0;
}


// *************************************************************************************************
// @fn          do_altitude_measurement
// @brief       Perform single altitude measurement
// @param       none
// @return      none
// *************************************************************************************************
void do_altitude_measurement(u8 filter)
{
	volatile u32 pressure;

	// If sensor is not ready, skip data read
	if ((PS_INT_IN & PS_INT_PIN) == 0) return;

	// Get pressure (format is 1Pa) from sensor
	pressure = ps_get_pa();

	// Store measured pressure value
	if (filter == FILTER_OFF)
	{
		sAlt.pressure = pressure;
	}
	else
	{
		// Filter current pressure
#ifdef FIXEDPOINT
        pressure = (u32)(((pressure * 2) + (sAlt.pressure * 8))/10);
#else
		pressure = (u32)((pressure * 0.2) + (sAlt.pressure * 0.8));
#endif
		// Store average pressure
		sAlt.pressure = pressure;
	}

	// Convert pressure (Pa) to altitude (m or ft).
	sAlt.altitude = conv_pa_to_altitude(sAlt.pressure);

#ifdef CONFIG_VARIO
	// Stash a copy to the vario after filtering. If doing so before, there
	// is just too much unnecessary fluctuation, up to +/- 7Pa seen.
	vario_p_write(pressure);
#endif
}


// *************************************************************************************************
// @fn          sx_altitude
// @brief       Altitude direct function. Sx restarts altitude measurement.
// @param       u8 line	LINE1, LINE2
// @return      none
// *************************************************************************************************
void sx_altitude(u8 line)
{
	sAlt.mode++;
	if (sAlt.mode >= ALTITUDE_LAST_MODE) {
		sAlt.mode = ALTITUDE_REGULAR;
	}
	display.flag.line1_full_update = 1;

	if (sAlt.mode == ALTITUDE_SKYDIVING) {
		fptr_lcd_function_line2(LINE2, DISPLAY_LINE_CLEAR);
		ptrMenu_L2 = &menu_L2_Altitude;
		display.flag.line2_full_update = 1;
		fptr_lcd_function_line2 = ptrMenu_L2->display_function;
	} else if (ptrMenu_L2 != menu_L2[menu_L2_position]) {
		fptr_lcd_function_line2(LINE2, DISPLAY_LINE_CLEAR);
		ptrMenu_L2 = menu_L2[menu_L2_position];
		display.flag.line2_full_update = 1;
		fptr_lcd_function_line2 = ptrMenu_L2->display_function;
	}
}


// *************************************************************************************************
// @fn          mx_altitude
// @brief       Mx button handler to set the altitude offset.
// @param       u8 line		LINE1
// @return      none
// *************************************************************************************************
void mx_altitude(u8 line)
{
	s32 altitude = sAlt.altitude;
	s32 limit_high = 9000, limit_low = -500;
	u8 units_symbol;

	// Clear display
	clear_display_all();

#ifdef CONFIG_ALTITUDE_UNIT_SETTABLE
	display_symbol(sys.flag.use_metric_units ? LCD_UNIT_L1_M : LCD_UNIT_L1_M, SEG_ON);
#elif defined(CONFIG_ALTITUDE_UNIT_METERS)
	display_symbol(LCD_UNIT_L1_M, SEG_ON);
#elif defined(CONFIG_ALTITUDE_UNIT_FEET)
	display_symbol(LCD_UNIT_L1_FT, SEG_ON);
#endif

	if (sAlt.mode == ALTITUDE_SKYDIVING) {
		sAlt.altitude = 0;
		update_pressure_table(0, sAlt.pressure);
		display.flag.line1_full_update = 1;
	} else {
		// Loop values until all are set or user breaks	set
		while(1)
		{
			// Idle timeout: exit without saving
			if (sys.flag.idle_timeout) break;

			// Button STAR (short): save, then exit
			if (button.flag.star)
			{
				update_pressure_table(altitude, sAlt.pressure);
				display.flag.line1_full_update = 1;
				break;
			}

			// Set current altitude - offset is set when leaving function
			set_value(&altitude, 4, 3, limit_low, limit_high,
				  SETVALUE_DISPLAY_VALUE + SETVALUE_FAST_MODE + SETVALUE_DISPLAY_ARROWS, LCD_SEG_L1_3_0, display_value1);
		}
	}

	// Clear button flags
	button.all_flags = 0;
}

void nx_altitude(u8 line) {
	sAlt.mode = ALTITUDE_REGULAR;

	ptrMenu_L2 = menu_L2[menu_L2_position];
	display.flag.line2_full_update = 1;
	fptr_lcd_function_line2 = ptrMenu_L2->display_function;

	menu_skip_next(line);
}

// *************************************************************************************************
// @fn          display_altitude
// @brief       Display routine. Supports display in meters and feet.
// @param       u8 line			LINE1
//		u8 update		DISPLAY_LINE_UPDATE_FULL, DISPLAY_LINE_UPDATE_PARTIAL, DISPLAY_LINE_CLEAR
// @return      none
// *************************************************************************************************
void display_altitude(u8 line, u8 update)
{
	if (update == DISPLAY_LINE_UPDATE_FULL)
	{
		sAlt.state = MENU_ITEM_VISIBLE;
		start_altitude_measurement();

		u8 m, ft;
#ifdef CONFIG_ALTITUDE_UNIT_SETTABLE
		if (sys.flag.use_metric_units) {
		       m = SEG_ON; ft = SEG_OFF;
		} else {
		       m = SEG_OFF; ft = SEG_ON;
		}
#elif defined(CONFIG_ALTITUDE_UNIT_METERS)
		m = SEG_ON; ft = SEG_OFF;
#elif defined(CONFIG_ALTITUDE_UNIT_FEET)
		m = SEG_OFF; ft = SEG_ON;
#endif
	       // Display "m" or "ft" symbol
	       display_symbol(LCD_UNIT_L1_M, m);
	       display_symbol(LCD_UNIT_L1_FT, ft);
	       display_symbol(LCD_SYMB_ARROW_UP, SEG_OFF);
	}
	if (update == DISPLAY_LINE_UPDATE_FULL || update == DISPLAY_LINE_UPDATE_PARTIAL) {
		// Update display only while measurement is active
		if (sAlt.timeout > 0) {
			u8 *str;
			if (sAlt.mode == ALTITUDE_SKYDIVING && sAlt.altitude > 1000) {
				u16 altitude = (sAlt.altitude + 50) / 100;
				str = _itoa(altitude, 3, 1);
				display_chars(LCD_SEG_L1_3_1, str, SEG_ON);
				display_symbol(LCD_SEG_L1_DP1, SEG_ON);
			} else {
				// Display altitude in xxxx m format, allow 3 leading blank digits
				if (sAlt.altitude >= 0) {
					str = _itoa(sAlt.altitude, 4, 3);
					display_symbol(LCD_SYMB_ARROW_UP, SEG_ON);
					display_symbol(LCD_SYMB_ARROW_DOWN, SEG_OFF);
				} else {
					str = _itoa(-sAlt.altitude, 4, 3);
					display_symbol(LCD_SYMB_ARROW_UP, SEG_OFF);
					display_symbol(LCD_SYMB_ARROW_DOWN, SEG_ON);
				}

				display_chars(LCD_SEG_L1_3_0, str, SEG_ON);
				display_symbol(LCD_SEG_L1_DP1, SEG_OFF);
			}
		}
	} else if (update == DISPLAY_LINE_CLEAR) {
		// Disable pressure measurement
		sAlt.state = MENU_ITEM_NOT_VISIBLE;

		// Stop measurement
		stop_altitude_measurement();

		// Clean up function-specific segments before leaving function
		display_symbol(LCD_UNIT_L1_M, SEG_OFF);
		display_symbol(LCD_UNIT_L1_FT, SEG_OFF);
		display_symbol(LCD_SYMB_ARROW_UP, SEG_OFF);
		display_symbol(LCD_SYMB_ARROW_DOWN, SEG_OFF);
		display_symbol(LCD_SEG_L1_DP1, SEG_OFF);
	}
}

#endif // FEATURE_ALTITUDE
