#include <Arduino.h>
#include <EEPROM.h>
#define MIN_STEP_LENGTH 8
#define MIN_STEP_LENGTH_S 0.000008
#define F_PI 3.1415926535897932384626433832795f
#define MAXIMUM_TABLE_SIZE 1500

const uint8_t step_pin = 11;
const uint8_t dir_pin = 10;

constexpr float controller_steps = 400.0f;
constexpr float ratio = 5.0f;
constexpr float steps_per_rev = controller_steps * ratio;
constexpr float step_angle = (2.0 * F_PI) / steps_per_rev;

float max_val = 20.0f * (F_PI / 180.0f);
float period = 0.1;

float manual_angle = 10.0f * (F_PI / 180.0f);

int table_length = 0;
float step_width_table[MAXIMUM_TABLE_SIZE] = {0};
char dir_table[MAXIMUM_TABLE_SIZE] = {0};

bool motor_running = false; //Motor status variable


void setTableParameters(float _max_val, float _period)
{
    max_val = _max_val;
    period = _period;
    int _table_length = floorf(_max_val / (2 * F_PI / (steps_per_rev))) * 4.0;
    if (_table_length < MAXIMUM_TABLE_SIZE)
    {
        table_length = _table_length;
    }
}

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

float alpha(float n)
{
    return n * ((2.0 * F_PI) / steps_per_rev);
}

void calculateTimeTable()
{
    for (int i = 0; i < table_length / 4; i++) // Generate first quarter
    {
        step_width_table[i] = asinf(alpha(float(i)) / alpha(float(table_length / 4))) * period / (2 * F_PI);
        dir_table[i] = 1;
    }

    for (int i = table_length / 4; i > 0; i--)
    {
        step_width_table[i] = step_width_table[i] - step_width_table[i - 1] - MIN_STEP_LENGTH_S;
        if (step_width_table[i] < 0)
            step_width_table[i] = 0;
    }

    {
        int j = table_length / 2 - 1;
        for (int i = 0; i <= table_length / 4; i++)
        {
            step_width_table[j] = step_width_table[i];
            dir_table[j] = -1;
            j--;
        }
    }

    for (int i = 0; i < table_length / 2; i++)
    {
        step_width_table[i + table_length / 2] = step_width_table[i];
        dir_table[i + table_length / 2] = -dir_table[i];
    }
}

void step_pulse()
{
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(MIN_STEP_LENGTH);
    digitalWrite(step_pin, LOW);
}

void move(float angle)
{
    float manual_time = 1.0;
    int steps_to_go = steps_per_rev * (angle / (2 * F_PI));
    unsigned long step_time_delay = static_cast<unsigned long>(manual_time * ((2 * F_PI) / (steps_per_rev * manual_angle)));
    digitalWrite(dir_pin, steps_to_go > 0 ? HIGH : LOW);
    for (int i = 0; i < steps_to_go; i++)
    {
        step_pulse();
        delayMicroseconds(step_time_delay);
    }
}

void handleSerial(uint8_t buffer)
{
    switch (buffer)
    {
    case 'c':
        motor_running = true;
        break;
    case 'd':
        motor_running = false;
        break;
    case 'g':
        manual_angle = Serial.parseFloat() * (F_PI / 180.0f);
        break;
    case 'e':
        move(manual_angle);
        break;
    case 'f':
        move(-manual_angle);
    default:
        break;
    }
    while(Serial.available() > 0)
    {
        uint8_t t = Serial.read();
    }
    delay(100);
}

void setup()
{
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    Serial.begin(1000000);
    delay(100);
    calculateTimeTable();
    setTableParameters(max_val, period);
}

// Loop Variables
unsigned long last_step_time = 0;
float pos = 0;
int index = 0;
//

void loop()
{
    if (motor_running)
    {
        unsigned long step_time = step_width_table[index] * 1e6f;
        char dir = dir_table[index];
        if ((micros() - last_step_time) >= step_time)
        {
            digitalWrite(dir_pin, dir > 0 ? HIGH : LOW);
            step_pulse();
            pos += dir > 0 ? step_angle : -step_angle;
            last_step_time = micros();
            index++;
            if (index == table_length)
                index = 0;
        }
        Serial.println(pos * ((180) / F_PI));
    }

    if (Serial.available() > 0)
    {
        uint8_t ser_buffer = Serial.read();
        if (ser_buffer != 10) // Linefeed
            handleSerial(ser_buffer);
    }
}