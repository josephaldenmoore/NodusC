/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

/*****************************************************************************
//!
//!                            Sensor Control
//!
*****************************************************************************/

// Create and initialize a sensor
void sensor_create();

// Power on a sensor
void sensor_power_on();

// Power off a sensor
void sensor_power_off();

// Put a sensor in sleep mode
void sensor_sleep_mode_set();

// Wake up a sensor
void sensor_sleep_mode_wake();

// Calibrate a sensor
void sensor_calibrade();

// Configure sensor
void sensor_config_set();

// read sensor config
void sensor_config_get();

// Write data to sensor
void sensor_write();

// Read data from sensor
void sensor_read();

/*****************************************************************************
//!
//!                            Radio Control
//!
*****************************************************************************/

// Power on radio interface
void radio_power_on();

// Power off radio interface
void radio_power_off();

// Put radio in sleep mode
void radio_sleep_mode_set();

// Wake radio from sleep
void radio_sleep_mode_wake();

// Send data over the radio interface
void radio_data_send();

// recv data over the radio interface
void radio_data_recv();

// Set the role of radio interface - master, slave, or dual
void radio_set_role();

/*****************************************************************************
//!
//!                            Task Management
//!
*****************************************************************************/

// Start a customer application task
void task_app_start();

// Stop a customer application task
void task_app_stop();

// Send a message from one costumer task to another
void task_msg_send();

// Receive a message from one costumer task to another
void task_msg_recv();

// Lock a task mutex
void task_mutex_lock();

// Unlock a task mutex
void task_mutex_unlock();

/*****************************************************************************
//!
//!                            Timer management
//!
*****************************************************************************/
// create a periodic or one shot timer
void timer_create();

// delete a periodic or one shot timer
void timer_delete();

// get notification when timer expires
void timer_event_notify();

/*****************************************************************************
//!
//!                            OTA upgrade
//!
*****************************************************************************/

// OTA upgrade customer app
void ota_upgrade_app();

// OTA upgrade platform SDK
void ota_upgrade_sdk();

/*****************************************************************************
//!
//!                            Security
//!
*****************************************************************************/

// Initiate pairing
void security_initiate_pairing();

// Initiate bonding
void security_initiate_bonding();

// Remove pairing from a peer device
void security_remove_pairing();

// remove bonding froma peer device
void security_remove_bonding();

// Set up OOB key
void security_setup_OOB_key();

// Delete up OOB key
void security_delete_OOB_key();

// Change up OOB key
void security_change_OOB_key();

/*****************************************************************************
//!
//!                            General
//!
*****************************************************************************/

// Return SDK version
void sdk_version_check();

// Notify customer application about a fatal error
void sdk_fatal_error_notify();

/* [] END OF FILE */
