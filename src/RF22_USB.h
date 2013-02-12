/*
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)
  Copyright 2012  Simon Foster (simon.foster [at] inbox [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for SerialToLCD.c.
 */

#ifndef _RF22_USB_H_
#define _RF22_USB_H_

    /* Includes: */
        #include <avr/io.h>
        #include <avr/wdt.h>
        #include <avr/interrupt.h>
        #include <avr/power.h>
        #include <stdlib.h>
        #include <stdbool.h>
//        #include <math.h>

        #include "Descriptors.h"
        #include "Lib/rfm23.h"

        #include <LUFA/Version.h>
        #include <LUFA/Drivers/USB/USB.h>
        #include <LUFA/Drivers/Board/LEDs.h>
		
	/* Macros: */
		#define COMMAND_ESCAPE     0x1B

    /* Function Prototypes: */
        void SetupHardware(void);

        void EVENT_USB_Device_Connect(void);
        void EVENT_USB_Device_Disconnect(void);
        void EVENT_USB_Device_ConfigurationChanged(void);
        void EVENT_USB_Device_ControlRequest(void);

#endif

