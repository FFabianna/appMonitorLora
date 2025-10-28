#ifndef CONN_H
#define CONN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef  __cplusplus
extern "C" {
#endif

    #define TAG_DLMS                    "DLMS"
    #define MAX_CHANNELS                32      // Adjust to set the maximum number of channels including time
    #define CHANNEL_OBIS_LENGTH         25      // Do not change

    typedef int (*serial_cll)(uint8_t *data, uint16_t size);
    typedef int (*serial_config_cll)(uint8_t modeDLMS, uint16_t baudrate);

    typedef enum {
        METER_MODEL_MT880
    } METER_MODEL;

    typedef enum {
        METER_PHASE_A = 0,
        METER_PHASE_B,
        METER_PHASE_C,
    } METER_PHASE;

    typedef enum {
        SECURITY_TYPE_NONE = 0,
        SECURITY_TYPE_AUTHENTICATED_MESSAGES = 0x10,
        SECURITY_TYPE_ENCRYPTED_MESSAGES = 0x20,
        SECURITY_TYPE_AUTHENTICATED_AND_ENCRYPTED_MESSAGES = 0x30
    } SECURITY_TYPE;

    typedef struct {
        METER_MODEL model;
        char pass[65];
        char master[65];
        char cipher[65];
        uint16_t serverLogAddress;
        bool useServerLogAddress;
        uint16_t serverPhyAddress;
        bool ignorePhyAddress;
        serial_cll uartRead;
        serial_cll uartWrite;
        serial_config_cll uartConfig;
        SECURITY_TYPE security;
        void *log;
        uint8_t addressSize;
        uint8_t retry;
    } METER_CONNECTION;

    typedef enum {
        DRIVER_ERROR_OK = 0,
        DRIVER_ERROR_PASSWORD = -1,
        DRIVER_ERROR_METER_NOT_RESPOND = -2,
        DRIVER_ERROR_READING_OBJECT = -3,
        DRIVER_ERROR_DATA_INVALID = -4,
        DRIVER_ERROR_METHOD = -5,
        DRIVER_ERROR_WRITE = -6,
        DRIVER_ERROR_NOT_SUPPORTED = -7,
        DRIVER_ERROR_OUT_OF_MEMORY = -8,
        DRIVER_ERROR_NOT_IMPLEMENTED = -9,
    } DRIVER_ERROR_CODES;

    typedef enum {
        METER_AUTH_TYPE_READING = 0,
        METER_AUTH_TYPE_WRITING,
    } METER_AUTH_TYPE;

    typedef enum {
        OBJECT_CLASS_TYPE_NONE = 0,
        OBJECT_CLASS_TYPE_DATA,
        OBJECT_CLASS_TYPE_REGISTER,
        OBJECT_CLASS_TYPE_EXTENDED_REGISTER,
        OBJECT_CLASS_TYPE_DEMAND_REGISTER,
    } OBJECT_CLASS_TYPE;

    typedef enum {
        ATTRIBUTE_INDEX_NAME_NONE = 0,
        ATTRIBUTE_INDEX_NAME_VALUE,
        ATTRIBUTE_INDEX_NAME_CURRENT,
        ATTRIBUTE_INDEX_NAME_LAST,
    } ATTRIBUTE_INDEX_TYPE;

    typedef enum {
        UNIT_TYPE_YEAR = 1,
        UNIT_TYPE_MONTH = 2,
        UNIT_TYPE_WEEK = 3,
        UNIT_TYPE_DAY = 4,
        UNIT_TYPE_HOUR = 5,
        UNIT_TYPE_MINUTE = 6,
        UNIT_TYPE_SECOND = 7,
        UNIT_TYPE_DEGREE = 8,
        UNIT_TYPE_DEGREE_CELSIUS = 9,
        UNIT_TYPE_LOCAL_CURRENCY = 10,
        UNIT_TYPE_METRE = 11,
        UNIT_TYPE_METRE_PER_SECONDS = 12,
        UNIT_TYPE_CUBIC_METRE = 13,
        UNIT_TYPE_CORRECTED_CUBIC_METRE = 14,
        UNIT_TYPE_CUBIC_METRE_PER_HOUR = 15,
        UNIT_TYPE_CORRECTED_CUBIC_METRE_PER_HOUR = 16,
        UNIT_TYPE_CUBIC_METRE_PER_DAY = 17,
        UNIT_TYPE_CORRECTED_CUBIC_METRE_PER_DAY = 18,
        UNIT_TYPE_LITRE = 19,
        UNIT_TYPE_KILOGRAM = 20,
        UNIT_TYPE_NEWTON = 21,
        UNIT_TYPE_NEWTON_METER = 22,
        UNIT_TYPE_PASCAL = 23,
        UNIT_TYPE_BAR = 24,
        UNIT_TYPE_JOULE = 25,
        UNIT_TYPE_JOULE_PER_HOUR = 26,
        UNIT_TYPE_WATT = 27,
        UNIT_TYPE_VOLT_AMPERE = 28,
        UNIT_TYPE_VAR = 29,
        UNIT_TYPE_WATT_HOUR = 30,
        UNIT_TYPE_VOLT_AMPERE_HOUR = 31,
        UNIT_TYPE_VAR_HOUR = 32,
        UNIT_TYPE_AMPERE = 33,
        UNIT_TYPE_COULOMB = 34,
        UNIT_TYPE_VOLT = 35,
        UNIT_TYPE_VOLT_PER_METRE = 36,
        UNIT_TYPE_FARAD = 37,
        UNIT_TYPE_OHM = 38,
        UNIT_TYPE_RESISTIVITY = 39,
        UNIT_TYPE_WEBER = 40,
        UNIT_TYPE_TESLA = 41,
        UNIT_TYPE_AMPERE_PER_METRE = 42,
        UNIT_TYPE_HENRY = 43,
        UNIT_TYPE_HERTZ = 44,
        UNIT_TYPE_R_SUB_W = 45,
        UNIT_TYPE_R_SUB_B = 46,
        UNIT_TYPE_R_SUB_S = 47,
        UNIT_TYPE_VOLT_SQUARED_HOURS = 48,
        UNIT_TYPE_AMPERE_SQUARED_HOURS = 49,
        UNIT_TYPE_KILOGRAM_PER_SECOND = 50,
        UNIT_TYPE_SIEMENS = 51,
        UNIT_TYPE_KELVIN = 52,
        UNIT_TYPE_R_SUB_U2H = 53,
        UNIT_TYPE_R_SUB_I2H = 54,
        UNIT_TYPE_R_SUB_V = 55,
        UNIT_TYPE_PERCENTAGE = 56,
        UNIT_TYPE_AMPERE_HOUR = 57,
        /* Reserved values */
        UNIT_TYPE_SIGNAL_STRENGTH_DB_MILLIWATT = 70,
        UNIT_TYPE_SIGNAL_STRENGTH_DB_MICROVOLT = 71,
        UNIT_TYPE_LOGARITHMIC_UNIC_DB = 72,
        /* Reserved values */
        UNIT_TYPE_NOT_UNIT = 255
    } UNIT_TYPE;

    typedef struct {
        char obis[CHANNEL_OBIS_LENGTH];
        OBJECT_CLASS_TYPE class;
        ATTRIBUTE_INDEX_TYPE attribute;
        int scale;
        UNIT_TYPE unit;
    } CHANNEL_DESCRIPTION;

    typedef struct {
        uint32_t datetime;
        char serial[64];
        char firmware[64];

        uint32_t *dateTimeOfInterval;
        float *valueOfInterval;
        uint16_t totalIntervals;
        char *obisChannels;
        CHANNEL_DESCRIPTION *channels;
        uint8_t totalChannels;
        int capturePeriod;       

        float voltage[3];
        float current[3];
        float frequency;
        float power_factor[3];
        
        float active_power_total_import;
        float active_power_total_export;
        float reactive_power_total_import;
        float reactive_power_total_export;
        float apparent_power_total;

        float active_power_import[3];
        float active_power_export[3];
        float reactive_power_import[3];
        float reactive_power_export[3];

        float reactive_power_total_qi;
        float reactive_power_total_qii;
        float reactive_power_total_qiii;
        float reactive_power_total_qiv;

        float active_energy_total_import;
        float active_energy_total_export;
        float reactive_energy_total_import;
        float reactive_energy_total_export;

        float voltageAngleB2A;
        float voltageAngleC2A;
    } METER_DATA;

   
    /* Inicializa la estructura de conexión */
    void init_params(
        METER_CONNECTION *conn
    );

    void init_data(
        METER_DATA *data
    );

    DRIVER_ERROR_CODES requestInstantMeasures(
        METER_CONNECTION *conn,
        METER_DATA *data
    );

    DRIVER_ERROR_CODES requestEnergyMeasures(
        METER_CONNECTION *conn,
        METER_DATA *data
    ); 

    DRIVER_ERROR_CODES requestDateTime(
        METER_CONNECTION *conn,
        METER_DATA *data
    );

    DRIVER_ERROR_CODES setDateTime(
        METER_CONNECTION *conn,
        uint32_t timestamp
    );

    DRIVER_ERROR_CODES requestFirmwareAndSerial(
        METER_CONNECTION *conn,
        METER_DATA *data
    );

    DRIVER_ERROR_CODES requestLoadProfile1(
        METER_CONNECTION *conn,
        METER_DATA *data,
        uint32_t dateTimeEnd,
        uint8_t hours
    );

    DRIVER_ERROR_CODES requestLoadProfile2(
        METER_CONNECTION *conn,
        METER_DATA *data,
        uint32_t dateTimeEnd,
        uint8_t hours
    );

    DRIVER_ERROR_CODES requestCapturePeriodLoadProfile1(
        METER_CONNECTION *conn,
        METER_DATA *data
    );
    
    DRIVER_ERROR_CODES requestCapturePeriodLoadProfile2(
        METER_CONNECTION *conn,
        METER_DATA *data
    );

    DRIVER_ERROR_CODES requestChannelsLoadProfile1(
        METER_CONNECTION *conn,
        METER_DATA *data
    );

    DRIVER_ERROR_CODES requestChannelsLoadProfile2(
        METER_CONNECTION *conn,
        METER_DATA *data
    );

#ifdef  __cplusplus
}
#endif
#endif //CONN_H
