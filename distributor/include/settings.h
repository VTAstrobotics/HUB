#ifndef SETTINGS_H
#define SETTINGS_H


/**
 * @defgroup drvMac Drive Constants
 * Constants and settings related to the drivetrain subsystem
 */
/**
 * @defgroup teleop Distributor Constants
 * Constants and settings related to the disributor nodes
 */

 namespace DistributorConsts {

    // General Settings
    /**
     * @ingroup distributor
     * @brief Amount of error to place in axis controls
     */
    const float DEADZONE_SIZE = 0.1f;

    /**
     * @brief Maximum forward/backward speed
     */
    const float MAX_LINEAR_VELOCITY = 1.0;

    /**
     * @brief Maximum rotational speed
     */
    const float MAX_ANGULAR_VELOCITY = 1.57;

    // Motor Configuration
    /**
     * @brief The number of motors in the drive system
     */
    const int DRIVE_M_COUNT = 2;

    /**
     * @brief Flag to invert the right drive motor logic
     */
    const bool INVERT_R_DRIVE = true; // Use bool just in case

}

#define DISTRIB_TOPIC "joy"

#define DPAD_ACTIVATION_DISTANCE 0.5

/* Key bindings for current robot (2023-2024) */
#define CTRL_STOP_SEQ_1 BUTTON_BACK
#define CTRL_STOP_SEQ_2 BUTTON_START
#define CTRL_STOP_SEQ_3 BUTTON_MANUFACTURER

/**********************************************************************
 *                                                                    *
 * Drive controls                                                     *
 *                                                                    *
 * RT drives forwards; LT drives backwards                            *
 * Left stick x-axis controls turning                                 *
 *                                                                    *
 **********************************************************************/
#define CTRL_DRIVE_FWD AXIS_RTRIGGER
#define CTRL_DRIVE_BCK AXIS_LTRIGGER
#define CTRL_DRIVE_TRN AXIS_LEFTX
#define CTRL_DRIVE_SPD BUTTON_A

/* Indexes for each button on xbox controller, NEVER USE DIRECTLY */
#define BUTTON_A 0
#define BUTTON_B 1
#define BUTTON_X 2
#define BUTTON_Y 3
#define BUTTON_BACK 6
#define BUTTON_MANUFACTURER 8
#define BUTTON_START 7
#define BUTTON_LSTICK 9
#define BUTTON_RSTICK 10
#define BUTTON_LBUMPER 4
#define BUTTON_RBUMPER 5

#define AXIS_LEFTX 0
#define AXIS_LEFTY 1
#define AXIS_RIGHTX 3
#define AXIS_RIGHTY 4
#define AXIS_LTRIGGER 2
#define AXIS_RTRIGGER 5
#define AXIS_DPAD_X 6
#define AXIS_DPAD_Y 7

#endif