#ifndef PROTOCOL_H
#define PROTOCOL_H


#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif


#include <stdint.h>
#include <stdbool.h>

#define PROTOCOL_MAGIC_NUMBER 0x56494152	// "VIAR" in little endian
#define PROTOCOL_VERSION 1

#define TCP_PORT 27654
#define UDP_PORT 27655

#define HEARTBEAT_INTERVAL_MS 10
#define MAX_AXES 6
#define NUMBER_OF_DOF MAX_AXES	// Number of degrees of freedom equals MAX_AXES

#ifndef YRC1000
#define MAX_ALARM_COUNT 16
// Duration type for trajectory timing
typedef struct {
    int32_t sec;		// seconds
    int32_t nanos;		// nanoseconds
} duration_t;
#else
#include "utils.h"
#endif


// Message types
typedef enum {
    MSG_TEST_TRAJECTORY_COMMAND = 0x01,
    MSG_TURN_SERVO_POWER_ON = 0x02,
    MSG_HEARTBEAT = 0x03,
    MSG_ROBOT_POSITION_VELOCITY_TORQUE = 0x04,
    MSG_OK = 0x05,
    MSG_ERROR = 0x06,
    MSG_TEST_ERROR_COMMAND = 0x07,	// Test command that always returns error
    MSG_GET_ERROR_INFO = 0x08,	// Get detailed error information
    MSG_ERROR_INFO = 0x09,	// Error information response (same structure as MSG_ERROR)
    MSG_ROBOT_STATUS = 0x0A,	// Robot status information
    MSG_REGISTER_UDP_PORT = 0x0B,	// Register UDP port for status messages
    MSG_RESET_ERRORS = 0x0C,	// Reset/clear all error conditions
    MSG_MOVE_GOAL = 0x0D,	// Move robot along trajectory with tolerance
    MSG_ECHO_TRAJECTORY = 0x0E,	// Echo back the received trajectory
    MSG_SET_MOTION_MODE = 0x0F,	// Set robot motion mode
    MSG_IS_IN_MOTION = 0x10,	// Query if robot is currently in motion
    MSG_STOP_MOTION = 0x11,	// Stop current motion
    MSG_GOAL_ACCEPTED = 0x12,	// Goal accepted with ID
    MSG_GOAL_STATUS = 0x13,	// Goal status update/notification
    MSG_CANCEL_GOAL = 0x14,	// Cancel a specific goal
    MSG_GET_GOAL_STATUS = 0x15,	// Query status of a goal
    MSG_FROM_JOINT_TO_CART = 0x16,
    MSG_FROM_CART_TO_JOINT = 0x17,
    MSG_GET_CART = 0x18
} message_type_t;

// Goal states
typedef enum {
    GOAL_STATE_PENDING = 0,	// Goal accepted but not started
    GOAL_STATE_ACTIVE = 1,	// Goal in progress
    GOAL_STATE_SUCCEEDED = 2,	// Goal reached successfully
    GOAL_STATE_CANCELLED = 3,	// Goal cancelled by client
    GOAL_STATE_ABORTED = 4	// Goal aborted due to error
} goal_state_t;

// Protocol header structure (18 bytes total)
typedef PACK(struct {
	     uint32_t magic_number;	// 4 bytes
	     uint8_t version;	// 1 byte
	     uint8_t message_type;	// 1 byte
	     uint64_t timestamp_ms;	// 8 bytes
	     uint32_t payload_length;	// 4 bytes
	     }) protocol_header_t;


// Status message payload structure
typedef PACK(struct {
	     int64_t timestamp;	// 8 bytes
	     uint8_t num_axes;	// 1 byte
	     double position[MAX_AXES];	// 8 * 8 = 64 bytes
	     double velocity[MAX_AXES];	// 8 * 8 = 64 bytes
	     double torque[MAX_AXES];	// 8 * 8 = 64 bytes
	     double position_corrected[MAX_AXES];	// 8 * 8 = 64 bytes
	     }) status_payload_t;


// Error message payload structure
typedef PACK(struct {
	     int32_t error_code;	// 4 bytes - error code
	     char message[252];	// 252 bytes - error message string (null-terminated)
	     }) error_payload_t;

// Robot status payload structure  
typedef PACK(struct {
	     int64_t ts;	// 8 bytes - timestamp
	     int mode;		// 4 bytes - robot mode
	     _Bool e_stopped;	// 1 byte - estop status
	     _Bool drives_powered;	// 1 byte - drive power status  
	     _Bool motion_possible;	// 1 byte - motion enabled
	     _Bool in_motion;	// 1 byte - motion status
	     _Bool in_error;	// 1 byte - error status
	     int error_codes[MAX_ALARM_COUNT + 1];	// (16+1)*4 = 68 bytes - error codes
	     int size;		// 4 bytes - number of active error codes
	     }) robot_status_payload_t;

// UDP port registration payload structure
typedef PACK(struct {
	     uint16_t udp_port;	// 2 bytes - UDP port for status messages
	     }) udp_port_registration_payload_t;

// Trajectory point structure for move goals
typedef PACK(struct {
	     double positions[NUMBER_OF_DOF];	// Joint positions
	     double velocities[NUMBER_OF_DOF];	// Joint velocities  
	     double accelerations[NUMBER_OF_DOF];	// Joint accelerations
	     double torque[NUMBER_OF_DOF];	// Joint torques
	     duration_t time_from_start;	// Time from trajectory start
	     }) trajectory_point_t;

// Tolerance structure (same as trajectory point without time)
typedef PACK(struct {
	     double positions[NUMBER_OF_DOF];	// Position tolerances
	     double velocities[NUMBER_OF_DOF];	// Velocity tolerances
	     double accelerations[NUMBER_OF_DOF];	// Acceleration tolerances
	     double torque[NUMBER_OF_DOF];	// Torque tolerances
	     }) tolerance_t;

// Move goal payload structure (uses malloc for dynamic allocation)
typedef struct {
    uint32_t number_of_axes_controlled;
    uint32_t group_index;
    uint32_t trajectory_size;	// Number of trajectory points
    trajectory_point_t *trajectory_data;	// Pointer to trajectory array (malloc allocated)
    uint32_t tolerance_size;	// Number of tolerance points  
    tolerance_t *tolerance_data;	// Pointer to tolerance array (malloc allocated)
} move_goal_payload_t;

// Motion mode payload structure
typedef PACK(struct {
    uint8_t motion_mode;	// 1 byte - motion mode value
}) motion_mode_payload_t;

// Boolean payload structure for commands that return true/false
typedef PACK(struct {
    uint8_t value;	// 1 byte - boolean value (0 = false, 1 = true)
}) boolean_payload_t;

// Cartesian coordinate
typedef PACK(struct 
{ 
	int group_id;
	double cartesianCoord[8];
}) cartesian_payload_t;

// Cartesian coordinate
typedef PACK(struct 
{ 
	int group_id;
	double positionAngleDegree[8];
}) position_angle_degree_payload_t;

// Goal accepted response payload
typedef PACK(struct {
    int32_t goal_id;		// 4 bytes - unique goal identifier (random)
    int64_t timestamp_ms;	// 8 bytes - when goal was accepted
}) goal_accepted_payload_t;

// Goal status payload
typedef PACK(struct {
    int32_t goal_id;		// 4 bytes - goal identifier
    uint8_t state;		// 1 byte - current goal state (goal_state_t)
    double progress;		// 8 bytes - completion percentage (0.0 to 1.0)
    int64_t timestamp_ms;	// 8 bytes - status timestamp
}) goal_status_payload_t;

// Cancel goal payload
typedef PACK(struct {
    int32_t goal_id;		// 4 bytes - goal to cancel
}) cancel_goal_payload_t;

// Group ID
typedef PACK(struct {
    int32_t group_id;  // 4 bytes - goup id
}) group_id_t;


typedef struct {
    protocol_header_t *header;
    void *payload;
} command_response_context_t;

#define MSG_OK_RESPONSE() allocate_response_context(0, MSG_OK)

#define MSG_ERR_RESPONSE(err)                                                                   \
    ({                                                                                          \
        command_response_context_t* _ctx = allocate_response_context(sizeof((err)), MSG_ERROR); \
        if ((_ctx) != NULL) {                                                                   \
            memcpy((_ctx)->payload, &(err), sizeof((err)));                                     \
        }                                                                                       \
        _ctx;                                                                                   \
    })

#define MSG_ERR_RESPONSE_INT(val) \
  ({ \
    int err = val; \
    MSG_ERR_RESPONSE(err);})

void free_move_goal(move_goal_payload_t * goal);
move_goal_payload_t *move_goal_from_payload(void *data, uint32_t size);
command_response_context_t *allocate_response_context(uint32_t length, uint8_t type);
void free_command_response_context(command_response_context_t * ctx);
move_goal_payload_t *allocate_move_goal(uint32_t trajectory_len, uint32_t tolerance_len);


#endif				// PROTOCOL_H
