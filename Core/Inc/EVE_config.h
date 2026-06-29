/*****************************************************************************
* @file			: EVE_config.h
* @short        :
******************************************************************************
*
* @details
*
******************************************************************************
*/


#ifndef _EVE_CONFIG_H_
#define _EVE_CONFIG_H_

// DISPLAY_RES == WQVGA

#define EVE_DISP_WIDTH 480 		// Active width of LCD display
#define EVE_DISP_HEIGHT 272 	// Active height of LCD display
#define EVE_DISP_HCYCLE 548 	// Total number of clocks per line
#define EVE_DISP_HOFFSET 43 	// Start of active line
//#define EVE_DISP_HOFFSET 0 	// Start of active line
#define EVE_DISP_HSYNC0 0 		// Start of horizontal sync pulse
#define EVE_DISP_HSYNC1 41 		// End of horizontal sync pulse
#define EVE_DISP_VCYCLE 292 	// Total number of lines per screen
#define EVE_DISP_VOFFSET 12 	// Start of active screen
//#define EVE_DISP_VOFFSET 0 	// Start of active screen
#define EVE_DISP_VSYNC0 0 		// Start of vertical sync pulse
#define EVE_DISP_VSYNC1 10 		// End of vertical sync pulse
#define EVE_DISP_PCLK 5 		// Pixel Clock
#define EVE_DISP_SWIZZLE 0 		// Define RGB output pins
#define EVE_DISP_PCLKPOL 1 		// Define active edge of PCLK
#define EVE_DISP_CSPREAD 0
#define EVE_DISP_DITHER 1

#endif /* _EVE_CONFIG_H_ */
