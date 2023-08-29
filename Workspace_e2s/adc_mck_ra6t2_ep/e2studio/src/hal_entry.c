#include "common_utils.h"
#include "adc_ep.h"


/*******************************************************************************************************************//**
 * @addtogroup r_adc_ep
 * @{
 **********************************************************************************************************************/
extern volatile bool b_ready_to_read;
void R_BSP_WarmStart(bsp_warm_start_event_t event);

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

/*******************************************************************************************************************//**
 * The RA Configuration tool generates main() and uses it to generate threads if an RTOS is used.  This function is
 * called by main() when no RTOS is used.
 * This is the main loop of the Application.
 **********************************************************************************************************************/
void hal_entry(void)
{
    fsp_err_t err = FSP_SUCCESS;
       fsp_pack_version_t version =   { RESET_VALUE };

       /* version get API for FLEX pack information */
       R_FSP_VersionGet (&version);

       /* Example Project information printed on the Console */
       APP_PRINT(BANNER_1);
       APP_PRINT(BANNER_2);
       APP_PRINT(BANNER_3, EP_VERSION);
       APP_PRINT(BANNER_4, version.version_id_b.major, version.version_id_b.minor, version.version_id_b.patch);
       APP_PRINT(BANNER_5);
       APP_PRINT(BANNER_6);
       APP_PRINT(BANNER_7);

       /* Menu for User Selection */
       APP_PRINT(MENUOPTION1);
       APP_PRINT(MENUOPTION2);
       APP_PRINT(MENUOPTION3);
       APP_PRINT(MENUOPTION4);
    /* TODO: add your own code here */
       while (true)
          {
              if (APP_CHECK_DATA)
              {
                  err = read_process_input_from_RTT ();

                  /* handle error */
                  if (FSP_SUCCESS != err)
                  { /* adc scan start or stop failed, cleanup the adc initialization */
                      deinit_adc_module ();
                      APP_ERR_TRAP(err);
                  }
              }

              /* read the adc output data and status */
              if (true == b_ready_to_read)
              {
                  err = adc_read_data();
              }

              /* handle error */
              if (FSP_SUCCESS != err)
              { /* adc reading failed, cleanup the adc initialization */
                  deinit_adc_module ();
                  APP_ERR_TRAP(err);
              }

          }
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
    }
}

#if BSP_TZ_SECURE_BUILD

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
#endif
