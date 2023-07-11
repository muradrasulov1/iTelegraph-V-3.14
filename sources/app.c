#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_hid.h"
#include "board.h"
#include <MKL46Z4.h>

#include "host_keyboard.h"
#include "fsl_common.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#include "app.h"
#include "board.h"
#include "utils.h"

// tsi-specific includes
#include "fsl_tsi_v4.h"
#include "fsl_debug_console.h"
#include "fsl_lptmr.h"
#include "clock_config.h"

#if ((!USB_HOST_CONFIG_KHCI) && (!USB_HOST_CONFIG_EHCI) && (!USB_HOST_CONFIG_OHCI) && (!USB_HOST_CONFIG_IP3516HS))
#error Please enable USB_HOST_CONFIG_KHCI, USB_HOST_CONFIG_EHCI, USB_HOST_CONFIG_OHCI, or USB_HOST_CONFIG_IP3516HS in file usb_host_config.
#endif

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 *
 *
 ******************************************************************************/

// all defines below are tsi-code-specific

#define PAD_TSI_ELECTRODE_1_NAME "E1"

/* TSI indication leds for electrode 1/2 */
#define LED_INIT() LED_RED_INIT(LOGIC_LED_OFF)
#define LED_INIT2() LED_GREEN_INIT(LOGIC_LED_OFF)
#define LED_TOGGLE() LED_RED_TOGGLE()
#define LED_TOGGLE2() LED_GREEN_TOGGLE()

/* LLWU module wakeup source index for TSI module */
#define LLWU_TSI_IDX 4U

/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
/* Define LPTMR microseconds counts value */
#define LPTMR_USEC_COUNT (250000U)
/* Define the delta value to indicate a touch event */
#define TOUCH_DELTA_VALUE 100U


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief host callback function.
 *
 * device attach/detach callback function.
 *
 * @param deviceHandle          device handle.
 * @param configurationHandle   attached device's configuration descriptor information.
 * @param eventCode             callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The application don't support the configuration.
 */
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode);

/*!
 * @brief application initialization.
 */
static void USB_HostApplicationInit(void);

extern void USB_HostClockInit(void);
extern void USB_HostIsrEnable(void);
extern void USB_HostTaskFn(void *param);
void BOARD_InitHardware(void);

// SW1 and SW3 interrupt enabler
void operate_switch_interrupts();

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief USB host mouse instance global variable */
//extern usb_host_mouse_instance_t g_HostHidMouse;
/*! @brief USB host keyboard instance global variable */
extern usb_host_keyboard_instance_t g_HostHidKeyboard;
usb_host_handle g_HostHandle;
tsi_calibration_data_t buffer; // tsi-specific var for calibration purposes

int change_modes = 0;

//Debounce variables
int touch = 0;
int dot_button = 0;
int dash_button = 0;


int in_c_to_m = 0; // variable indicating the current mode of the system  1:C->M  ||  0:M->C
int tree_index; // used in binary tree for M->C decoding


// below variablea are SW1, SW2 specific
const int RED_LED_PIN = 29;
const int GREEN_LED_PIN = 5;
const int SWITCH_1_PIN = 3; //SW1
const int SWITCH_2_PIN = 12; //SW2
SIM_Type* global_SIM = SIM;
PORT_Type* global_PORTE = PORTE;
PORT_Type* global_PORTD = PORTD;
GPIO_Type* global_PTE = PTE;
GPIO_Type* global_PTD = PTD;

PORT_Type* global_PORTC = PORTC;
GPIO_Type* global_PTC = PTC;

const int binary_tree_arr[63] = { // used for the M->C implementation
		0, //None
		8, //E
		23, //T
		12, //I
		4, //A
		17, //N
		16, //M
		22, //S
		24, //U
		21, //R
		26, //W
		7, //D
		14, //K
		10, //G
		18, //O
		11, //H
		25, //V
		9, //F
		0, //None
		15, //L
		0, //None
		19, //P
		13, //J
		5, //B
		27, //X
		6, //C
		28, //Y
		29, //Z
		20, //Q
		0, //None
		0, //None
		34, //5
		33, //4
		0, //None
		32, //3
		0, //None
		0, //None
		0, //None
		31, //2
		0, //None
		0, //None
		0, //None
		0, //None
		0, //None
		0, //None
		0, //None
		30, //1
		35, //6
		0, //None
		0, //None
		0, //None
		0, //None
		0, //None
		0, //None
		0, //None
		36, //7
		0, //None
		0, //None
		0, //None
		37, //8
		0, //None
		38, //9
		39 //0
		};


/*******************************************************************************
 * Code
 ******************************************************************************/
#if defined(USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI > 0U)
void USB0_IRQHandler(void)
{
    USB_HostKhciIsrFunction(g_HostHandle);
}
#endif /* USB_HOST_CONFIG_KHCI */

void toggleMode(void)
{
					LEDGreen_Toggle(); /* Toggle the touch event indicating LED */
	            	if (in_c_to_m){ // reset the M->C specific variable
						in_c_to_m = 0;
						tree_index = 0;
						PRINTF("\r\nIn Morse to Char mode!\r\n");
					}
					else{
						in_c_to_m = 1;
						clearWord();
						LEDRed_Off();
						PRINTF("\r\nIn Char to Morse mode!\r\n");

					}
	            	change_modes = 0; //reset the mode switching variable

}

void TSI0_IRQHandler(void) // tsi-specific interrupt || crucial for mode switching
{

    if (TSI_GetMeasuredChannelNumber(TSI0) == BOARD_TSI_ELECTRODE_1)
    {
        if (TSI_GetCounter(TSI0) > (uint16_t)(buffer.calibratedData[BOARD_TSI_ELECTRODE_1] + TOUCH_DELTA_VALUE))
        {
        	// accessed in case the touch sensor trigger event occurred
        	change_modes += 1; //switch from C->M to M->C or the other way around
        	if (change_modes == 1){
            	touch = 1; //
        	}
        }
    }

    /* Clear flags */
    TSI_ClearStatusFlags(TSI0, kTSI_EndOfScanFlag);
    TSI_ClearStatusFlags(TSI0, kTSI_OutOfRangeFlag);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void operate_switch_interrupts() { //interrupt-backed switches operation
	NVIC_EnableIRQ(PORTC_PORTD_IRQn); // configure NVIC so that interrupt is enabled
}
void PORTC_PORTD_IRQHandler(void) {

	if ((PORTC->PCR[SWITCH_1_PIN] & PORT_PCR_ISF(1)) != 0){ // in case a dot is entered by pressing SW1
		dot_button = 1;
		PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_ISF(1); // clear the interrupt status flag by writing 1 to it
	}
	else{ // in case a dash is entered by pressing SW2
		dash_button = 1;
		PORTC->PCR[SWITCH_2_PIN] |= PORT_PCR_ISF(1); // clear the interrupt status flag by writing 1 to it
	}
}


void USB_HostClockInit(void)
{
#if defined(USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI > 0U)
    SystemCoreClockUpdate();
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
#endif
}

void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;
#if defined(USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI > 0U)
    uint8_t usbHOSTKhciIrq[] = USB_IRQS;
    irqNumber = usbHOSTKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];
#endif /* USB_HOST_CONFIG_KHCI */

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

void USB_HostTaskFn(void *param)
{
#if defined(USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI > 0U)
    USB_HostKhciTaskFunction(param);
#endif
}

/*!
 * @brief USB isr function.
 */


void BOARD_InitHardware(void){  // this function takes care of the Red and Green LEDs, along with the SW1 and SW2 initialization
	//Switches Initialization (SW1 and SW2)
	global_SIM = global_SIM;

	global_PORTE = global_PORTE;
	global_PORTD = global_PORTD;
	global_PORTC = global_PORTC;

	global_PTE = global_PTE;
	global_PTD = global_PTD;
	global_PTC = global_PTC;



	// RED LED Initialization
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //Enable the clock to port E
	PORTE->PCR[RED_LED_PIN] = PORT_PCR_MUX(0b001); //Set up PTE29 as GPIO
	PTE->PDDR |= GPIO_PDDR_PDD(1 << RED_LED_PIN); // make it output
	PTE->PSOR |= GPIO_PSOR_PTSO(1 << RED_LED_PIN); // turn off R LED

	// GREEN LED Initialization
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;  //Enable the clock to port D
	PORTE->PCR[GREEN_LED_PIN] = PORT_PCR_MUX(0b001); //Set up PTE5 as GPIO
	PTD->PDDR |= GPIO_PDDR_PDD(1 << GREEN_LED_PIN); // make it output
	PTD->PSOR |= GPIO_PSOR_PTSO(1 << GREEN_LED_PIN); // turn off G LED


	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //Enable the clock to port C (used by both SW1 and SW2)

	// setup switch 1
	PORTC->PCR[SWITCH_1_PIN] &= ~PORT_PCR_MUX(0b111); // Clear PCR Mux bits for PTC3
	PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_MUX(0b001); // Set up PTC3 as GPIO
	PTC->PDDR &= ~GPIO_PDDR_PDD(1 << SWITCH_1_PIN); // make it input
	PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_PE(1); // Turn on the pull enable
	PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_PS(1); // Enable the pullup resistor
	PORTC->PCR[SWITCH_1_PIN] &= ~PORT_PCR_IRQC(0b1111); // Clear IRQC bits for PTC3
	PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_IRQC(0b1001); // Set up the IRQC to interrupt on either edge (i.e. from high to low or low to high)


	// setup switch 2
 	PORTC->PCR[SWITCH_2_PIN] &= ~PORT_PCR_MUX(0b111); // Clear PCR Mux bits for PTC12
	PORTC->PCR[SWITCH_2_PIN] |= PORT_PCR_MUX(0b001); // Set up PTC12 as GPIO
	PTC->PDDR &= ~GPIO_PDDR_PDD(1 << SWITCH_2_PIN); // make it input
	PORTC->PCR[SWITCH_2_PIN] |= PORT_PCR_PE(1); // Turn on the pull enable
	PORTC->PCR[SWITCH_2_PIN] |= PORT_PCR_PS(1); // Enable the pullup resistor
	PORTC->PCR[SWITCH_2_PIN] &= ~PORT_PCR_IRQC(0b1111); // Clear IRQC bits for PTC12
	PORTC->PCR[SWITCH_2_PIN] |= PORT_PCR_IRQC(0b1001); // Set up the IRQC to interrupt on either edge (i.e. from high to low or low to high)

}



//Keyboard-specific code concerned with the host-keyboard connection
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle, // responsible for handling events like connecting the keyboard and receving packets of info
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode)
{    usb_status_t status1;
    usb_status_t status = kStatus_USB_Success;

    switch (eventCode)
    {
        case kUSB_HostEventAttach:
            status1 = USB_HostHidKeyboardEvent(deviceHandle, configurationHandle, eventCode);
            if ((status1 == kStatus_USB_NotSupported)) //&& (status2 == kStatus_USB_NotSupported))
            {
                status = kStatus_USB_NotSupported;
            }
            break;

        case kUSB_HostEventNotSupported:
            usb_echo("device not supported.\r\n");
            break;

        case kUSB_HostEventEnumerationDone:
            status1 = USB_HostHidKeyboardEvent(deviceHandle, configurationHandle, eventCode);
            if ((status1 != kStatus_USB_Success)) //&& (status2 != kStatus_USB_Success))
            {
                status = kStatus_USB_Error;
            }
            break;

        case kUSB_HostEventDetach:
            status1 = USB_HostHidKeyboardEvent(deviceHandle, configurationHandle, eventCode);
            if ((status1 != kStatus_USB_Success))//&& (status2 != kStatus_USB_Success))
            {
                status = kStatus_USB_Error;
            }
            break;

        default:
            break;
    }
    return status;
}

static void USB_HostApplicationInit(void) // SDK-provided function || initializing the host and initializing and enabling the access to the clock
{
    usb_status_t status = kStatus_USB_Success;

    USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        usb_echo("host init error\r\n");  // notify in case of the connection failure with the keyboard
        return;
    }
    USB_HostIsrEnable();
}




int main(void)
{

	BOARD_InitHardware(); // initialize the Switches (SW1 and SW2) and the respective interrupts
	tsi_config_t tsiConfig_normal = {0};
	lptmr_config_t lptmrConfig;
	memset((void *)&lptmrConfig, 0, sizeof(lptmrConfig));

	LPTMR_GetDefaultConfig(&lptmrConfig);
	/* TSI default hardware configuration for normal mode */
	TSI_GetNormalModeDefaultConfig(&tsiConfig_normal);

	/* Initialize the LPTMR */
	LPTMR_Init(LPTMR0, &lptmrConfig);

	/* Initialize the TSI */
	TSI_Init(TSI0, &tsiConfig_normal);

	/* Set timer period */
	LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

	NVIC_EnableIRQ(TSI0_IRQn);
	TSI_EnableModule(TSI0, true); /* Enable module */


	// enabling the SW1 and SW2 combined interrupt
	operate_switch_interrupts();

	PRINTF("\r\nTSI_V4 Normal_mode Example Start!\r\n");
	/*********  TSI SENSOR CALIBRATION PROCESS ************/
	memset((void *)&buffer, 0, sizeof(buffer));
	TSI_Calibrate(TSI0, &buffer);



	/********** SOFTWARE TRIGGER SCAN USING INTERRUPT METHOD ********/
	PRINTF("\r\nNOW, comes to the software trigger scan using interrupt method!\r\n");
	TSI_EnableModule(TSI0, false); //disabling the interrupt temporarily to set up the miscs
	TSI_EnableHardwareTriggerScan(TSI0, false); /* Enable software trigger scan */
	TSI_EnableInterrupts(TSI0, kTSI_GlobalInterruptEnable);
	TSI_EnableInterrupts(TSI0, kTSI_EndOfScanInterruptEnable);
	TSI_ClearStatusFlags(TSI0, kTSI_EndOfScanFlag);
	TSI_SetMeasuredChannelNumber(TSI0, BOARD_TSI_ELECTRODE_1);
	TSI_EnableModule(TSI0, true); //enabling the interrupt once all the miscs are set up
	LPTMR_StartTimer(LPTMR0); /* Start LPTMR triggering */

	// keyboard specific initialization functions
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    LED_Initialize();
    USB_HostApplicationInit();
    // end of keyboard specific funcs


	TSI_StartSoftwareTrigger(TSI0); // starts the timer, interrupted by a touch
	TSI_IsScanInProgress(TSI0); // once touched, IsInProgress toggles, signifies this run is over
	TSI_StartSoftwareTrigger(TSI0); // starts the timer, interrupted by a touch

	while(1){ // the execution is infinite | interrupted only by the reset button | allows for unlimited mode switching

		TSI_StartSoftwareTrigger(TSI0); // starts the timer, interrupted by a touch
		TSI_IsScanInProgress(TSI0); // once touch is recorded, IsInProgress toggles, signifying this run is over
		USB_HostTaskFn(g_HostHandle);
		USB_HostHidKeyboardTask(&g_HostHidKeyboard);

		//Debouncing the touch and SW1, SW2 sensors using delays
		if (touch)
		{
			// responding to a touch sensor trigger (switching the mode we are in)
			toggleMode();
			delay(); // added to avoid corrupted and unpredictable sensor data (debouncing the sensor data)
			touch = 0;
		}
		else if (dot_button) // determining the next tree index based on the dot input
		{
			tree_index = in_c_to_m ? tree_index : 2*tree_index + 1; // if dot -> go to the 2*i + 1 index in the binary tree
			short_delay(); // added to avoid corrupted and unpredictable sensor data (debouncing the switch button data)
			dot_button = 0;
		}
		else if (dash_button) // determining the next tree index based on the dash input
		{
			tree_index = in_c_to_m ? tree_index : 2*tree_index + 2; // if dot -> go to the 2*i + 2 index in the binary tree
			short_delay(); // added to avoid corrupted and unpredictable sensor data (debouncing the switch button data)
			dash_button = 0;
		}
	}
}
